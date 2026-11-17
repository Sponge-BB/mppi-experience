"""Run existing map reader tests against only an isolated localhost master."""
import hashlib
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import xml.etree.ElementTree as ET
import xmlrpc.client

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
BINARY = ROOT / "experiments/online_remani_validation/results/map_compute_20261110/after_test"
LIBRARY = ROOT / "devel/lib/libplan_env.so"
FILTER = (
    "GridMapImmutable.*:"
    "GridMapDistanceCopy.CompleteScopeKeepsValuesVersionAndFusionStampPinned:"
    "GridMapIncremental.AsyncCallbackAndUnchangedReusePreservePinnedSnapshot"
)
env = os.environ.copy()
env.pop("ROS_HOSTNAME", None)
env.pop("REMANI_MAP_BENCHMARK", None)
env.update(
    ROS_MASTER_URI="http://127.0.0.1:11461",
    ROS_IP="127.0.0.1",
    ROS_HOME=str(HERE / "isolated_test_ros_home"),
    ROS_LOG_DIR=str(HERE / "isolated_test_ros_logs"),
    PYTHONNOUSERSITE="1",
    OPENBLAS_NUM_THREADS="1",
    OMP_NUM_THREADS="1",
    LD_LIBRARY_PATH=str(ROOT / "devel/lib") + ":" + env.get("LD_LIBRARY_PATH", ""),
)
socket.setdefaulttimeout(.5)
summary = {
    "master_uri": env["ROS_MASTER_URI"],
    "binary": str(BINARY),
    "library": str(LIBRARY),
    "binary_sha256": hashlib.sha256(BINARY.read_bytes()).hexdigest(),
    "library_sha256": hashlib.sha256(LIBRARY.read_bytes()).hexdigest(),
    "filter": FILTER,
    "compiled": False,
    "motion_publishers_started": False,
}
master = None
child = None
started = time.monotonic()
try:
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 11461))  # Refuse an occupied port.
    with (HERE / "export_snapshot_test_master.log").open("w") as master_log:
        master = subprocess.Popen(
            ["/opt/ros/noetic/bin/roscore", "-p", "11461"], env=env,
            stdout=master_log, stderr=subprocess.STDOUT, start_new_session=True,
        )
        for _ in range(100):
            if master.poll() is not None:
                raise RuntimeError("isolated master exited")
            try:
                reply = xmlrpc.client.ServerProxy(env["ROS_MASTER_URI"]).getPid("/export_snapshot_test")
                if reply[0] == 1:
                    break
            except (OSError, xmlrpc.client.Error):
                pass
            time.sleep(.1)
        else:
            raise RuntimeError("isolated master did not become ready")
        with (HERE / "export_snapshot_behavior_tests.log").open("w") as log:
            child = subprocess.Popen([
                str(BINARY), "--gtest_filter=" + FILTER,
                "--gtest_output=xml:" + str(HERE / "export_snapshot_behavior_tests.xml"),
            ], env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            summary["test_pid"] = child.pid
            summary["peak_rss_kib"] = 0
            test_started = time.monotonic()
            while child.poll() is None:
                try:
                    for line in Path("/proc/%d/status" % child.pid).read_text().splitlines():
                        if line.startswith("VmHWM:"):
                            summary["peak_rss_kib"] = max(summary["peak_rss_kib"], int(line.split()[1]))
                except FileNotFoundError:
                    pass
                if time.monotonic() - test_started > 90:
                    raise TimeoutError("isolated map behavior tests exceeded 90s")
                time.sleep(.05)
            summary["returncode"] = child.returncode
            summary["test_elapsed_s"] = time.monotonic() - test_started
        xml = ET.parse(HERE / "export_snapshot_behavior_tests.xml").getroot()
        summary["tests"] = int(xml.attrib["tests"])
        summary["failures"] = int(xml.attrib["failures"])
        summary["disabled"] = int(xml.attrib.get("disabled", 0))
        summary["passed"] = summary["returncode"] == 0 and summary["failures"] == 0 and summary["tests"] > 0
except BaseException as error:
    summary["error"] = type(error).__name__ + ": " + str(error)
    summary["passed"] = False
finally:
    for process in (child, master):
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
    summary["elapsed_s"] = time.monotonic() - started
    (HERE / "export_snapshot_behavior_tests_summary.json").write_text(
        json.dumps(summary, indent=2) + "\n")
print(json.dumps(summary, indent=2))
raise SystemExit(0 if summary.get("passed") else 1)
