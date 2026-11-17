"""Small offline CPU profile; real q/dq, synthetic clear ESDF/reference, no ROS."""
import os
for key in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ[key] = "1"
import ast
import cProfile
import hashlib
import io
import json
from pathlib import Path
import pstats
import resource
import sys
import time

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
MODULES = ROOT / "src/mm_remani/scripts/mppi"
sys.path.insert(0, str(MODULES))
from active_arm_model import PiperArmModel
from active_arm_mppi import WholeBodyMPPI, Snapshot
from standby_whole_body_mppi import StandbyWholeBodyMPPI

runtime = json.loads((HERE / "after_restart_runtime_config.json").read_text())["params"]
config = runtime["/whole_body_mppi"][2]
model_params = runtime["/mm_controller_node"][2]
model = PiperArmModel.from_parameters(model_params)
model.lower[5] = max(model.lower[5], -.35)
model.upper[5] = min(model.upper[5], .35)
records = []
with (HERE / "joint_lead_fullrate.jsonl").open() as stream:
    for line in stream:
        record = json.loads(line)
        if record.get("topic") == "/joint_states_single" and len(record.get("dq", [])) == 6:
            records.append(record)
zero = next(record for record in records if max(map(abs, record["dq"])) == 0.)
pulse = max(records, key=lambda record: max(map(abs, record["dq"])))
paired_zero = dict(pulse, dq=[0.] * 6)

# Compile only the unmodified numerical delivery-check function, avoiding all
# ROS imports, node construction, publisher construction and callback threads.
node_source = ROOT / "src/mm_remani/scripts/whole_body_mppi.py"
node_tree = ast.parse(node_source.read_text(), filename=str(node_source))
checked_node = next(node for node in node_tree.body
                    if isinstance(node, ast.FunctionDef) and node.name == "checked_first_step")
namespace = {"np": np}
exec(compile(ast.Module(body=[checked_node], type_ignores=[]), str(node_source), "exec"), namespace)
checked_first_step = namespace["checked_first_step"]

now = pulse["stamp"]
header = np.zeros(32)
header[[0, 1, 4, 11, 15, 16, 19, 22, 23]] = [3., 1., 1., .05, .35, 2., 1., 1., 1.]
header[3] = header[18] = now
header[5:8] = [82, 82, 60]
header[8:11] = [-2., -2., 0.]
snapshot = Snapshot(header, np.full((82, 82, 60), 3.), [])
limits = config["base_limits"]
H = config["horizon"]
dt = 1. / config["rate_hz"]
arm_reference = model.q_nominal.copy()
navigation_reference = np.column_stack((np.linspace(0., 1., 21), np.zeros(21),
                                         np.zeros(21), np.full(21, limits["vmax"])))
standby_reference = np.zeros((2, 4))

def make_kernel(kind):
    cls = WholeBodyMPPI if kind == "navigation" else StandbyWholeBodyMPPI
    return cls(model, limits, (model.lower, model.upper, config["arm_velocity"],
                              config["arm_acceleration"]), config["postures"],
               enabled=True, seed=11, horizon=H, step=dt,
               random_samples=config["random_samples"], arm_position_servo=False)

def describe_profile(profiler, total_s):
    stats = pstats.Stats(profiler)
    def functions(name, filename=None):
        return [(key, value) for key, value in stats.stats.items()
                if key[2] == name and (filename is None or key[0].endswith(filename))]
    def one(name, filename=None):
        values = functions(name, filename)
        cumulative = sum(value[3] for _, value in values)
        return {"calls": sum(value[1] for _, value in values),
                "cumulative_ms": cumulative * 1000,
                "fraction_of_profile_wall": cumulative / total_s if total_s else None}
    groups = {
        "advance_including_base_small_numpy": one("advance", "active_arm_mppi.py"),
        "external_spheres_fk": one("spheres", "active_arm_mppi.py"),
        "ESDF_query_no_dynamic_points": one("query", "active_arm_mppi.py"),
        "configuration_collision_including_proxy": one("configuration_gap", "active_arm_mppi.py"),
    }
    groups["other"] = {"cumulative_ms": max(0., total_s*1000-sum(
        item["cumulative_ms"] for item in groups.values()))}
    nested = {
        "advance_base": one("_advance_base"),
        "configuration_gaps_batch": one("configuration_gaps_batch"),
        "self_gaps": one("self_gaps"),
        "shoulder_wrist_proxy_pair_gaps": one("pair_gaps"),
        "proposals_includes_advance": one("proposals"),
        "evaluate_includes_all_acceptance": one("evaluate"),
    }
    text = io.StringIO()
    pstats.Stats(profiler, stream=text).strip_dirs().sort_stats("cumulative").print_stats(30)
    return groups, nested, text.getvalue()

summary = {
    "source_feedback": "joint_lead_fullrate.jsonl",
    "model_parameters": "after_restart_runtime_config.json params /mm_controller_node",
    "model_equivalence": "same Piper URDF/proxy and 47 fixed + 20 arm spheres; joint6 runtime bounds retained",
    "configuration": {"horizon": H, "step_s": dt, "random_samples": config["random_samples"],
                      "base_limits": limits, "arm_velocity": config["arm_velocity"],
                      "arm_acceleration": config["arm_acceleration"], "arm_execution_model": "velocity_legacy"},
    "synthetic_inputs": {"base_state": "x=y=yaw=v=w=0; not a recorded base pose",
                         "reference": "navigation straight 1m at configured vmax; standby anchor at origin; nominal original standby arm pose",
                         "ESDF": "82x82x60 at .05m, constant 3m clear distance; NOT a recorded scene",
                         "dynamic_points": "none: no dynamic KDTree costs included"},
    "no_ROS_or_hardware": True,
    "single_thread_env": {key: os.environ[key] for key in
                          ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS")},
    "source_sha256": {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                      for path in (MODULES / "active_arm_mppi.py", MODULES / "active_arm_model.py",
                                   MODULES / "standby_whole_body_mppi.py", node_source)},
    "cases": [],
    "limitations": ["Synthetic clear scene and reference cannot reproduce live candidate selection or certify safety.",
                    "No ROS scheduling, snapshot decode, dynamic tree query, publisher or callback lock contention is measured.",
                    "cProfile adds overhead. Use unprofiled durations for absolute timing and the profile for function attribution.",
                    "Small sample count is intentional: this is a local cost diagnosis, not a latency percentile benchmark."],
}
started = time.monotonic()
for kind in ("navigation", "standby"):
    reference = navigation_reference if kind == "navigation" else standby_reference
    for label, record in (("recorded_zero_dq", zero), ("recorded_peak_dq", pulse),
                          ("peak_q_with_dq_zero_counterfactual", paired_zero)):
        if time.monotonic()-started > 8.:
            raise RuntimeError("offline profile loop budget reached")
        state = np.r_[np.zeros(5), record["q"], record["dq"]]
        kernel = make_kernel(kind)
        kernel.plan(state, reference, arm_reference, snapshot, now)  # one warmup
        timings = []; validation = []; errors = []
        for _ in range(3):
            begin = time.perf_counter()
            result, info = kernel.plan(state, reference, arm_reference, snapshot, now)
            timings.append((time.perf_counter()-begin)*1000)
            if result is not None:
                begin = time.perf_counter()
                try:
                    checked_first_step(kernel, state, result[0][0], snapshot, now)
                    error = None
                except ValueError as exc:
                    error = str(exc)
                validation.append((time.perf_counter()-begin)*1000)
                errors.append(error)
        profiler = cProfile.Profile()
        begin = time.perf_counter()
        result, info = profiler.runcall(kernel.plan, state, reference, arm_reference, snapshot, now)
        profiled_s = time.perf_counter()-begin
        name = kind + "_" + label
        profiler.dump_stats(str(HERE / ("mppi_cpu_" + name + ".pstats")))
        groups, nested, profile_text = describe_profile(profiler, profiled_s)
        (HERE / ("mppi_cpu_" + name + ".txt")).write_text(profile_text)
        summary["cases"].append({
            "kernel": kind, "feedback_case": label, "record": record,
            "unprofiled_plan_ms": timings,
            "unprofiled_first_command_check_ms": validation,
            "first_command_check_errors": errors,
            "profiled_plan_ms": profiled_s*1000,
            "profile_groups_disjoint": groups, "nested_context_do_not_sum": nested,
            "plan_result": {key: info[key] for key in ("mode", "candidates", "feasible", "intervention", "selection") if key in info},
        })
summary["loop_elapsed_s"] = time.monotonic()-started
summary["peak_rss_kib"] = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
(HERE / "mppi_real_feedback_cpu_profile.json").write_text(json.dumps(summary, indent=2)+"\n")
for case in summary["cases"]:
    print(case["kernel"], case["feedback_case"], "plan_ms", [round(v,2) for v in case["unprofiled_plan_ms"]],
          "delivery_check_ms", [round(v,2) for v in case["unprofiled_first_command_check_ms"]],
          "groups", {key:round(value["cumulative_ms"],2) for key,value in case["profile_groups_disjoint"].items()},
          "result",case["plan_result"])
print("loop_elapsed_s", round(summary["loop_elapsed_s"],3), "peak_rss_kib", summary["peak_rss_kib"])
