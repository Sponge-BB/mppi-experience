#!/usr/bin/env python3
"""Private wrapper of the unchanged UART driver with source-expiring inputs."""

import importlib.util
import json
import os
from pathlib import Path
import threading
import time
from urllib.parse import urlparse

import rospy
from std_msgs.msg import Bool, String

from base_lease_gate import BaseLeaseGate, validate_readiness


RELEASE = Path(__file__).resolve().parents[1]
NAMESPACE = "/standby_trial_20261105_integration_04/base"
spec = importlib.util.spec_from_file_location(
    "frozen_original_base_driver", RELEASE / "driver_reference/base_control.py")
original = importlib.util.module_from_spec(spec)
spec.loader.exec_module(original)


class _MonotonicTimer:
    """Keep original callbacks periodic when ROS/wall clock moves backwards."""

    def __init__(self, period, callback, reset=True):
        self.interval = period.to_sec()
        if not 0 < self.interval <= 1.0:
            raise ValueError("invalid original driver timer interval")
        self._stop = threading.Event()
        self._callback = callback
        self._thread = threading.Thread(target=self._run, name="leased_base_timer", daemon=True)
        self._thread.start()

    def _run(self):
        next_tick = time.monotonic() + self.interval
        while not self._stop.wait(max(0.0, next_tick - time.monotonic())):
            self._callback(None)
            # Do not burst old callbacks after an operating-system stall.
            next_tick += self.interval
            now = time.monotonic()
            if next_tick <= now:
                next_tick = now + self.interval

    def shutdown(self):
        self._stop.set()


class _OriginalRosAdapter:
    """Adapt only the private copied module; the global rospy stays untouched."""

    def __init__(self, ros):
        self.ros = ros

    def __getattr__(self, name):
        return getattr(self.ros, name)

    def init_node(self, *_args, **_kwargs):
        if self.ros.get_name() != NAMESPACE + "/leased_base_driver":
            raise RuntimeError("base driver requires its exact isolated namespace")

    def get_param(self, name, default=None):
        fixed = {"~write_timeout": .005, "~send_rate": 50.0,
                 "~command_timeout": .15, "~shutdown_zero_frames": 3}
        if name in fixed:
            requested = self.ros.get_param(name, fixed[name])
            if type(requested) not in (int, float) or requested != fixed[name]:
                raise ValueError(name + " must equal " + str(fixed[name]))
            return fixed[name]
        return self.ros.get_param(name, default)

    def Publisher(self, name, *args, **kwargs):
        if name == "/my_car_vel":
            name = NAMESPACE + "/feedback"
        return self.ros.Publisher(name, *args, **kwargs)

    def Subscriber(self, name, *args, **kwargs):
        if name == "/cmd_vel":
            # Never register the original unleased input, even briefly.
            return None
        raise RuntimeError("unexpected subscriber in original base driver")

    def Timer(self, period, callback, reset=True):
        return _MonotonicTimer(period, callback, reset=reset)


class LeasedBaseDriver(original.CmdVelToUartNode):
    def __init__(self):
        rospy.init_node("leased_base_driver", anonymous=False)
        if rospy.get_name() != NAMESPACE + "/leased_base_driver":
            raise RuntimeError("base driver requires its exact isolated namespace")
        master = urlparse(os.environ.get("ROS_MASTER_URI", ""))
        if master.hostname not in ("localhost", "127.0.0.1") or master.port != 11461:
            raise RuntimeError("base trial requires private localhost master 11461")
        if rospy.get_param("/use_sim_time", False):
            raise RuntimeError("hardware base driver refuses simulated ROS time")
        connect = rospy.get_param("~connect_transport", False)
        allow = rospy.get_param("~allow_motion", False)
        if type(connect) is not bool or type(allow) is not bool:
            raise ValueError("connect_transport and allow_motion must be bool")
        self.readiness_path = RELEASE / "config/hardware_readiness.json"
        self.gate = BaseLeaseGate(allow_motion=allow and connect)
        self.transport_connected = False
        self.ser = None
        if connect:
            ready, reason = validate_readiness(self.readiness_path)
            if not ready:
                raise RuntimeError(reason)
            # All UART framing, RX parsing, serial timeouts, 50 Hz timers and
            # shutdown zeros come from the byte-identical original driver.
            # Their timer callbacks use a monotonic scheduler in the facade.
            original.rospy = _OriginalRosAdapter(rospy)
            super().__init__()
            self.transport_connected = self.ser is not None and self.ser.is_open
            if not self.transport_connected:
                self.gate.stop("transport_disconnected")
                return
        else:
            rospy.on_shutdown(self.shutdown_hook)
        self.command_sub = rospy.Subscriber(NAMESPACE + "/leased_command", String,
                                           self.leased_callback, queue_size=1,
                                           tcp_nodelay=True)
        self.enable_sub = rospy.Subscriber(NAMESPACE + "/enable_flag", Bool,
                                          self.enable_callback, queue_size=1)
        self.status_pub = rospy.Publisher(NAMESPACE + "/gate_status", String, queue_size=1)
        self.status_timer = rospy.Timer(rospy.Duration(.1), self.publish_status)

    def leased_callback(self, msg):
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError):
            self.gate.stop("invalid_command_json")
            return
        self.gate.command(payload, time.monotonic(), time.time())

    def enable_callback(self, msg):
        if msg.data:
            ready, reason = validate_readiness(self.readiness_path)
            if not ready or not self.transport_connected:
                self.gate.stop(reason if not ready else "transport_disconnected")
                return
        self.gate.enable(msg.data, time.monotonic(), time.time())
        self.publish_status(None)

    def cmd_vel_callback(self, _msg):
        # Defence in depth: even direct use cannot revive an unleased input.
        self.gate.stop("legacy_cmd_vel_rejected")
        return False

    def _command_for_tx(self, now_monotonic=None):
        now = time.monotonic() if now_monotonic is None else now_monotonic
        return self.gate.output(now, time.time())

    def _write_velocity_frame(self, velocity, log_errors=True):
        result = super()._write_velocity_frame(velocity, log_errors)
        if not result:
            self.gate.stop("uart_write_failed")
        return result

    def send_timer_callback(self, event):
        # A concurrent disable cannot finish between selection and UART write.
        # The existing UART write is bounded by write_timeout = 5 ms.
        with self.gate.lock:
            return super().send_timer_callback(event)

    def publish_status(self, _event):
        self.gate.output(time.monotonic(), time.time())
        state = self.gate.status()
        state["transport_connected"] = self.transport_connected
        self.status_pub.publish(String(data=json.dumps(state, sort_keys=True)))

    def shutdown_hook(self):
        with self.gate.lock:
            self.gate.stop("shutdown")
            if hasattr(self, "status_timer"):
                self.status_timer.shutdown()
            if self.ser is not None:
                super().shutdown_hook()


if __name__ == "__main__":
    LeasedBaseDriver()
    rospy.spin()
