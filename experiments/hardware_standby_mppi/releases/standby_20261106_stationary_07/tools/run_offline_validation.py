#!/usr/bin/env python3
"""One bounded synthetic MPPI -> guard -> recording SDK check. No device I/O.

This is an integration check, not a real-robot experiment or a long avoidance
simulation. The original SDK protocol is tested separately with a mocked bus.
"""

import os

for _name in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS",
              "NUMEXPR_NUM_THREADS"):
    os.environ[_name] = "1"

import argparse
from dataclasses import asdict
import html
import json
from pathlib import Path
import resource
import sys
import time

RELEASE = Path(__file__).resolve().parents[1]
for _directory in ("core", "runtime"):
    sys.path.insert(0, str(RELEASE / _directory))

import numpy as np
from scipy.spatial import cKDTree

from active_arm_model import PiperArmModel
from active_arm_mppi import Snapshot
from hardware_command_guard import GuardConfig
from mppi_command_gateway import (InputCohort, MppiCommandGateway,
                                  NativeLeaseValidator, PlanEnvelope,
                                  RecordingSink)
from piper_lease_gate import JOINT_NAMES, PiperLeaseGate
from standby_whole_body_mppi import StandbyWholeBodyMPPI

Q_START = np.deg2rad([0., 110., -60., 0., 30., 0.])
Q_RETRACT = np.deg2rad([0., 140., -60., 0., 15., 0.])


class FakeClock:
    """Deterministic clock: computing for longer never ages synthetic inputs."""

    def __init__(self, monotonic=10., wall=1000.):
        self.monotonic = monotonic
        self.wall = wall

    def __call__(self):
        return self.monotonic, self.wall

    def advance(self, seconds):
        self.monotonic += seconds
        self.wall += seconds


class RecordingSDK:
    """No SDK instance, CAN device, socket, ROS publisher or motor is created."""

    def __init__(self):
        self.calls = []

    def EnableArm(self, *args):
        self.calls.append(("EnableArm", args))

    def MotionCtrl_1(self, *args):
        self.calls.append(("MotionCtrl_1", args))

    def MotionCtrl_2(self, *args):
        self.calls.append(("MotionCtrl_2", args))

    def JointCtrl(self, *args):
        self.calls.append(("JointCtrl", args))


def make_fixture():
    """Release-local native geometry and explicitly synthetic observation."""
    model = PiperArmModel.from_parameters(RELEASE / "model/parameters.yaml",
                                         RELEASE / "model/mm.urdf")
    config = GuardConfig(joint_lower=tuple(model.lower),
                         joint_upper=tuple(model.upper))
    limits = json.loads((RELEASE / "config/limits.json").read_text())
    # Match the kernel's effective joint bounds to the hardware guard, including
    # the narrower joint-6 bound inherited from the original real-robot bridge.
    arm_limits = [config.joint_lower, config.joint_upper,
                  model.velocity_limits, model.acceleration_limits]
    kernel = StandbyWholeBodyMPPI(model, limits, arm_limits, [Q_RETRACT],
                                 seed=11, random_samples=6)
    header = np.zeros(32)
    header[:2] = [3, 1]
    header[2:8] = [1, 1000., 1, 120, 120, 50]
    header[8:12] = [-2., -3., -.1, .05]
    header[12] = 67
    header[14:20] = [1, .35, 3., 1000., 1000., 1]
    header[20] = 1
    # This free field and one moving point are a test fixture. They do not
    # claim that unseen real LiDAR space is free or resolve sensor coverage.
    snapshot = Snapshot(header, np.full((120, 120, 50), 10., dtype=np.float32),
                        [(np.array([0., .1, 0.]),
                          cKDTree(np.array([[.14, -.14, 1.065]])))])
    state = np.r_[np.zeros(5), Q_START, np.zeros(6)]
    reference = np.array([[0., 0., 0., 0.]])
    return model, config, kernel, snapshot, state, reference


def make_envelopes(state, control, source_stamp=1000.05, sequence=1,
                   cohort_expiry=1000.12):
    cohort = InputCohort(cohort_id="synthetic-observation-1",
                         state=tuple(float(x) for x in state),
                         state_stamp=1000., obstacle_stamp=1000.,
                         expires_at=cohort_expiry, frame_id="odom")
    plan = PlanEnvelope(sequence=sequence, source_stamp=source_stamp,
                        expires_at=source_stamp + .15,
                        cohort_id=cohort.cohort_id,
                        control=tuple(float(x) for x in control))
    return cohort, plan


def run_validation():
    started = time.monotonic()
    model, config, kernel, snapshot, state, reference = make_fixture()
    clock = FakeClock()
    sink = RecordingSink()
    validator = NativeLeaseValidator(kernel)
    gateway = MppiCommandGateway(config, validator, sink, clock)
    sdk = RecordingSDK()
    driver = PiperLeaseGate(sdk, allow_motion=True)
    if not gateway.update_feedback(tuple(state), clock.wall):
        raise RuntimeError("synthetic gateway feedback was rejected")
    if not gateway.set_enabled(True):
        raise RuntimeError("synthetic gateway enable was rejected")
    if not driver.update_feedback(JOINT_NAMES, tuple(state[5:11]), clock.wall,
                                  clock.monotonic, clock.wall):
        raise RuntimeError("synthetic driver feedback was rejected")
    if not driver.enable(True, clock.monotonic, clock.wall):
        raise RuntimeError("synthetic driver enable was rejected")
    clock.advance(.05)
    planned, diagnostics = kernel.plan(state, reference, Q_START, snapshot,
                                       clock.wall)
    if planned is None:
        raise RuntimeError("synthetic MPPI scenario found no feasible plan")
    controls, prediction = planned
    cohort, envelope = make_envelopes(state, controls[0])
    if not gateway.submit(envelope, cohort, snapshot):
        raise RuntimeError("gateway rejected synthetic plan: %r" % gateway.status())
    if len(sink.packets) != 1:
        raise RuntimeError("gateway did not emit exactly one command")
    packet = sink.packets[0]
    accepted = driver.command(JOINT_NAMES, packet.joint_positions,
                              packet.sequence, packet.source_stamp,
                              clock.monotonic, clock.wall,
                              expires_at=packet.expires_at)
    if not accepted:
        raise RuntimeError("mock driver rejected packet: " + driver.reason)
    # A watchdog tick does not emit or renew a control command.
    clock.advance(.01)
    gateway.tick()
    if len(sink.packets) != 1:
        raise RuntimeError("watchdog unexpectedly republished a command")
    original_expiry = driver.lease_expiry
    clock.advance(.061)
    gateway.tick()
    driver.tick(clock.monotonic, clock.wall)
    if driver.enabled or not sink.stops:
        raise RuntimeError("original input deadline did not stop the chain")
    report = {
        "schema": "standby-offline-mppi-pipeline-v1",
        "release": RELEASE.name,
        "synthetic_only": True,
        "hardware_motion_tested": False,
        "real_sdk_instantiated": False,
        "ros_started": False,
        "native_spheres": int(len(model.body_spheres(Q_START))),
        "case": "moving synthetic point approaches extended arm; base remains a free control",
        "fixture": {"initial_joint_deg": np.rad2deg(Q_START).tolist(),
                    "moving_point_m": [.14, -.14, 1.065],
                    "point_velocity_m_s": [0., .1, 0.],
                    "distance_grid_bytes": int(snapshot.distance.nbytes)},
        "mppi": {"first_control": controls[0].tolist(),
                 "diagnostics": diagnostics,
                 "predicted_base_displacement_m": float(np.linalg.norm(prediction[-1, :2])),
                 "predicted_joint_change_rad": (prediction[-1, 5:11] - Q_START).tolist()},
        "input_cohort": asdict(cohort),
        "plan": asdict(envelope),
        "packet": asdict(packet),
        "standard_base_twist": packet.twist(),
        "lease_metadata": packet.lease_metadata(),
        "joint_goal_delta_rad": (np.asarray(packet.joint_positions) - Q_START).tolist(),
        "recorded_packet_count_after_watchdog": len(sink.packets),
        "recorded_gateway_stops": sink.stops,
        "mock_sdk_calls": sdk.calls,
        "driver_original_monotonic_expiry": original_expiry,
        "driver_final_status": driver.status(),
        "checks": {"real_mppi_plan_generated": True,
                   "bounded_command_rechecked": True,
                   "explicit_original_deadline_preserved": abs(packet.expires_at-cohort.expires_at) < 1e-8,
                   "watchdog_did_not_republish": len(sink.packets) == 1,
                   "mock_driver_latched_stopped": not driver.enabled},
        "elapsed_s": time.monotonic()-started,
        "process_peak_rss_mib": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024.,
        "limits": ["Synthetic point and free grid, not real sensing or localization.",
                   "Native sphere geometry and kinematic checks, not physical position-response validation.",
                   "One leased command and timeout, not a complete obstacle-avoidance trajectory.",
                   "A deterministic test clock is used; real callback timing and the 20 ms recheck budget are not validated.",
                   "No ROS, CAN or base serial transport is opened."],
    }
    if not all(report["checks"].values()):
        raise RuntimeError("an offline integration check failed")
    return report


def write_overview(report, path):
    """Small SVG without plotting libraries, GUI state or extra processes."""
    desired = report["mppi"]["first_control"]
    packet = report["packet"]
    bounded = [packet["base_v"], packet["base_w"]] + list(packet["joint_velocities"])
    rows = []
    for index, (label, request, sent) in enumerate(zip(
            ["base v (m/s)", "base w (rad/s)"] + ["joint %d (rad/s)" % i for i in range(1, 7)],
            desired, bounded)):
        y = 169 + index*34
        rows.append('<text x="40" y="%d">%s</text>' % (y, label))
        rows.append('<text x="290" y="%d">%+.6f</text>' % (y, request))
        rows.append('<text x="490" y="%d" fill="#147a51">%+.6f</text>' % (y, sent))
        rows.append('<rect x="645" y="%d" width="%.3f" height="10" fill="#147a51"/>' %
                    (y-10, min(160., abs(sent)*800.)))
    expiry = packet["expires_at"]
    footer = "One packet only; original expiry %.3f s; timeout latched the mock driver stopped." % expiry
    svg = '''<svg xmlns="http://www.w3.org/2000/svg" width="900" height="630" viewBox="0 0 900 630">
<rect width="900" height="630" fill="#f7f9fc"/>
<g font-family="sans-serif" font-size="16" fill="#193047">
<text x="35" y="42" font-size="25" font-weight="bold">MPPI to reused driver: offline integration</text>
<text x="35" y="74" fill="#a33d19">SYNTHETIC ONLY - no hardware movement, no ROS or CAN</text>
<text x="35" y="107">Native 67-sphere model; moving point near extended arm; base is allowed to move.</text>
<text x="40" y="135" font-weight="bold">Coordinate</text>
<text x="290" y="135" font-weight="bold">MPPI first control</text>
<text x="490" y="135" font-weight="bold">After guard + recheck</text>
%s
<line x1="35" y1="454" x2="865" y2="454" stroke="#bfcdd9"/>
<text x="35" y="485">%s</text>
<text x="35" y="517">SDK calls were recorded in memory; joint targets use the original rad-to-millidegree conversion.</text>
<text x="35" y="549">This checks software interfaces and expiry handling, not a real obstacle-avoidance result.</text>
<text x="35" y="581">Peak process RSS: %.1f MiB; one solver plan; numerical library threads: 1.</text>
</g></svg>''' % ("\n".join(rows), html.escape(footer), report["process_peak_rss_mib"])
    path.write_text(svg)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path,
                        default=RELEASE / "reports/offline_validation")
    args = parser.parse_args()
    report = run_validation()
    args.output.mkdir(parents=True, exist_ok=True)
    report_path = args.output / "report.json"
    report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    write_overview(report, args.output / "overview.svg")
    print(json.dumps({"passed": True, "synthetic_only": True,
                      "report": str(report_path),
                      "peak_rss_mib": report["process_peak_rss_mib"]}))


if __name__ == "__main__":
    main()
