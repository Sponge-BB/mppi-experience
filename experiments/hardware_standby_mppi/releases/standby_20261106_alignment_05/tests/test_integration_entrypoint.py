"""Three ROS wiring checks with in-memory messages; no ROS master or devices."""
import json
from pathlib import Path
import sys
import threading
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

RELEASE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RELEASE / "runtime"))
import standby_integration_node as entry
from mppi_command_gateway import CommandPacket, RecordingSink


class Message:
    def __init__(self, data=None):
        self.data = data


class JointState:
    def __init__(self):
        self.header = SimpleNamespace(seq=0, stamp=None)
        self.name, self.position, self.velocity, self.effort = [], [], [], []


class FakeROS:
    Time = SimpleNamespace(from_sec=lambda value: SimpleNamespace(to_sec=lambda: value))

    def __init__(self):
        self.events = []

    def Publisher(self, topic, kind, **kwargs):
        return SimpleNamespace(publish=lambda msg: self.events.append((topic, msg)))

    def Subscriber(self, *args, **kwargs):
        return SimpleNamespace()

    def get_param(self, name, default=None):
        return default

    def on_shutdown(self, callback):
        self.shutdown_callback = callback


class IntegrationEntrypointTests(unittest.TestCase):
    def setUp(self):
        modules = {}
        for package, messages in (("sensor_msgs", {"JointState": JointState}),
                                  ("std_msgs", {"Bool": Message, "String": Message})):
            parent, child = ModuleType(package), ModuleType(package + ".msg")
            for name, kind in messages.items():
                setattr(child, name, kind)
            parent.msg = child
            modules[package], modules[package + ".msg"] = parent, child
        context = patch.dict(sys.modules, modules)
        context.start()
        self.addCleanup(context.stop)
        self.ros = FakeROS()
        self.packet = CommandPacket(7, 1000., 1000.12, .01, -.02,
                                    (0., 1., -1., 0., .5, 0.), (.01,) * 6)

    def test_publishes_original_metadata_then_position_and_base(self):
        with patch.object(entry.time, "monotonic", return_value=10.), \
                patch.object(entry.time, "time", return_value=1000.02):
            sink = entry.RosCommandSink(self.ros)
            for driver in ("arm", "base"):
                sink.on_status(driver, Message('{"enabled": true}'))
            sink.send(self.packet)
        events = self.ros.events
        self.assertEqual([topic for topic, _ in events], [
            entry.NAMESPACE + "/arm/command_lease",
            entry.NAMESPACE + "/arm/joint_ctrl_single",
            entry.NAMESPACE + "/base/leased_command"])
        metadata = json.loads(events[0][1].data)
        joint = events[1][1]
        self.assertEqual(metadata, self.packet.lease_metadata())
        self.assertEqual(joint.header.seq, self.packet.sequence)
        self.assertEqual(joint.header.stamp.to_sec(), self.packet.source_stamp)
        self.assertEqual(tuple(joint.position), self.packet.joint_positions)
        self.assertEqual(joint.velocity, [])
        self.assertEqual(joint.effort, [])
        self.assertEqual(json.loads(events[2][1].data),
                         dict(metadata, base_v=.01, base_w=-.02))

    def test_missing_disabled_or_stale_driver_status_cannot_publish(self):
        with patch.object(entry.time, "monotonic", return_value=10.), \
                patch.object(entry.time, "time", return_value=1000.02):
            sink = entry.RosCommandSink(self.ros)
            for statuses in ({}, {"arm": (True, 10.), "base": (False, 10.)},
                             {"arm": (True, 9.8), "base": (True, 10.)}):
                with self.subTest(statuses=statuses):
                    sink.status = statuses
                    with self.assertRaisesRegex(RuntimeError, "driver_status"):
                        sink.send(self.packet)
        self.assertEqual(self.ros.events, [])

    def test_default_records_only_and_base_launch_uses_expected_namespace(self):
        launch = ET.parse(RELEASE / "config/integration.launch").getroot()
        self.assertEqual(launch.find("arg[@name='execute']").get("default"), "false")
        group = launch.find("group")
        base = group.find("node[@name='leased_base_driver']")
        namespace = base.get("ns", "")
        if not namespace.startswith("/"):
            namespace = group.get("ns").rstrip("/") + "/" + namespace
        self.assertEqual(namespace, entry.NAMESPACE + "/base")

        def fake_shadow_init(node, release, namespace):
            node.rospy, node.world, node.config = self.ros, "odom", {}
            node.model = SimpleNamespace(lower=(-2.,)*6, upper=(2.,)*6)
            node.kernel, node.lock = None, threading.RLock()

        with patch.object(entry.HardwareShadowNode, "__init__", fake_shadow_init), \
                patch.object(entry, "NativeLeaseValidator"), \
                patch.object(entry, "MppiCommandGateway"), \
                patch.object(entry.threading, "Thread"), \
                patch.object(entry, "RosCommandSink") as transport:
            node = entry.HardwareIntegrationNode()
            self.assertFalse(node.execute)
            self.assertIsInstance(node.sink, RecordingSink)
            transport.assert_not_called()


if __name__ == "__main__":
    unittest.main()
