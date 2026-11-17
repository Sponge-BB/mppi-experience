#!/usr/bin/env python3
"""Optional, explicitly started CAN receiver: no SDK, TX, motor or interface setup.

Only recvmsg() is used on an already-up CAN interface.  Kernel CAN filtering
selects three feedback IDs.  This is receive-only APPLICATION code; it does not
change the CAN adapter into electrical listen-only mode (normal hardware CAN
acknowledgements remain a property of the already configured controller).

Importing this module does not open a socket or import ROS.  main() refuses any
ROS master except 192.168.137.128:11462 and never starts a master or a driver.
"""

import argparse
from dataclasses import asdict
import json
import math
import os
import re
import socket
import struct
import time
from urllib.parse import urlparse

from piper_feedback_decode import (CAN_EFF_FLAG, CAN_ERR_FLAG, CAN_RTR_FLAG,
                                   CAN_SFF_MASK, CLASSIC_CAN_FRAME, PAIR_IDS,
                                   JointFeedbackAssembler)


SOL_CAN_RAW = getattr(socket, "SOL_CAN_RAW", 101)
CAN_RAW_FILTER = getattr(socket, "CAN_RAW_FILTER", 1)
CAN_RAW_RECV_OWN_MSGS = getattr(socket, "CAN_RAW_RECV_OWN_MSGS", 4)
SO_TIMESTAMPNS = getattr(socket, "SO_TIMESTAMPNS", 35)
TIMESPEC = struct.Struct("@ll")


def exact_feedback_filters():
    # CAN_ERR_FLAG in can_mask has a routing meaning inside SocketCAN: it
    # selects the error receive list rather than normal data-frame matching.
    # Error delivery stays disabled by the default CAN_RAW_ERR_FILTER=0.
    # The pure decoder independently rejects CAN_ERR_FLAG for defence in depth.
    mask = CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG
    return b"".join(struct.pack("=II", can_id, mask) for can_id in PAIR_IDS)


def receive_stamp_from_ancillary(ancillary, now_wall, now_monotonic, max_age=.15):
    """Preserve kernel receive age; userspace dequeue time cannot freshen data."""
    for level, kind, data in ancillary:
        if level == socket.SOL_SOCKET and kind == SO_TIMESTAMPNS:
            if len(data) < TIMESPEC.size:
                raise ValueError("short kernel receive timestamp")
            sec, nsec = TIMESPEC.unpack(data[:TIMESPEC.size])
            if not 0 <= nsec < 1000000000:
                raise ValueError("invalid kernel nanoseconds")
            age = now_wall - (sec + nsec * 1e-9)
            if not math.isfinite(age) or age < -.005 or age > max_age:
                raise ValueError("kernel timestamp is stale or from the future")
            return now_monotonic - max(0., age)
    raise ValueError("kernel receive timestamp missing")


class ReceiveOnlySocket:
    """Small receive-only interface, with no public transmit method."""

    def __init__(self, interface="can0", timeout=.10):
        if not re.fullmatch(r"[A-Za-z0-9_.-]{1,15}", interface):
            raise ValueError("invalid CAN interface name")
        self.interface = interface
        self.timeout = float(timeout)
        if not math.isfinite(self.timeout) or self.timeout <= 0.:
            raise ValueError("invalid receive timeout")
        self._socket = None

    def open(self):
        if self._socket is not None:
            raise RuntimeError("receiver already open")
        receiver = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            receiver.setsockopt(SOL_CAN_RAW, CAN_RAW_FILTER, exact_feedback_filters())
            receiver.setsockopt(SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, 0)
            receiver.setsockopt(socket.SOL_SOCKET, SO_TIMESTAMPNS, 1)
            receiver.settimeout(self.timeout)
            receiver.bind((self.interface,))
        except Exception:
            receiver.close()
            raise
        self._socket = receiver

    def receive(self):
        if self._socket is None:
            raise RuntimeError("receiver is not open")
        frame, ancillary, flags, _ = self._socket.recvmsg(
            CLASSIC_CAN_FRAME.size, socket.CMSG_SPACE(TIMESPEC.size))
        if flags & (socket.MSG_TRUNC | socket.MSG_CTRUNC):
            raise ValueError("truncated CAN frame or timestamp")
        stamp = receive_stamp_from_ancillary(ancillary, time.time(), time.monotonic())
        if len(frame) != CLASSIC_CAN_FRAME.size:
            raise ValueError("not a classic CAN frame")
        can_id, dlc, payload = CLASSIC_CAN_FRAME.unpack(frame)
        if dlc != 8:
            raise ValueError("wrong joint frame DLC")
        return can_id, payload, stamp

    def close(self):
        if self._socket is not None:
            self._socket.close()
            self._socket = None


def require_isolated_master(uri):
    parsed = urlparse(uri)
    if (parsed.scheme != "http" or parsed.hostname != "192.168.137.128"
            or parsed.port != 11462):
        raise ValueError("receive-only trial requires existing isolated LAN ROS master 11462")


def main():
    import rospy
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", default="can0")
    parser.add_argument("--duration", type=float, default=30.)
    parser.add_argument("--output-jsonl")
    args = parser.parse_args(rospy.myargv()[1:])
    if not math.isfinite(args.duration) or args.duration <= 0.:
        parser.error("duration must be a finite positive observation interval")
    require_isolated_master(os.environ.get("ROS_MASTER_URI", ""))
    import rospy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    rospy.init_node("piper_can_receive_only", anonymous=False)
    if rospy.get_param("/use_sim_time", False):
        raise RuntimeError("real feedback receiver refuses simulated ROS time")
    publisher = rospy.Publisher("/standby_trial_20261105_feedback_02/sensors/joint_states",
                               JointState, queue_size=1)
    status_publisher = rospy.Publisher("/standby_trial_20261105_feedback_02/feedback_status",
                                       String, queue_size=1)
    assembler = JointFeedbackAssembler()
    receiver = ReceiveOnlySocket(args.interface)
    record = open(args.output_jsonl, "x") if args.output_jsonl else None
    start = time.monotonic()
    last_status = start - 1.
    try:
        receiver.open()
        while not rospy.is_shutdown() and time.monotonic() - start < args.duration:
            try:
                can_id, payload, stamp = receiver.receive()
                assembler.ingest(can_id, payload, stamp)
            except socket.timeout:
                pass
            except ValueError as error:
                rospy.logwarn_throttle(1., "Ignored CAN feedback: %s", error)
            now = time.monotonic()
            snapshot = assembler.snapshot(now)
            if snapshot is not None:
                message = JointState()
                message.header.stamp = rospy.Time.now() - rospy.Duration.from_sec(
                    max(0., now - snapshot.stamp_monotonic))
                message.header.frame_id = "standby_feedback_02_base_link"
                message.name = ["joint%d" % index for index in range(1, 7)]
                message.position = list(snapshot.positions)
                message.velocity = list(snapshot.velocities)
                publisher.publish(message)
                if record:
                    record.write(json.dumps({"event": "joint_feedback", "elapsed": now - start,
                                             **asdict(snapshot)}) + "\n")
                    record.flush()
            if now - last_status >= .5:
                status = {"receive_only_application": True, "can_tx_calls": 0,
                          "interface": args.interface,
                          "accepted_frames": assembler.accepted_frames,
                          "rejected_frames": assembler.rejected_frames,
                          "published_snapshots": assembler.published_snapshots,
                          "state": assembler.last_reason,
                          "velocity_source": "position_difference_filtered",
                          "kernel_receive_timestamp_required": True}
                status_publisher.publish(String(data=json.dumps(status)))
                if record:
                    record.write(json.dumps({"event": "receiver_status", **status}) + "\n")
                    record.flush()
                last_status = now
    finally:
        receiver.close()
        if record:
            record.close()


if __name__ == "__main__":
    main()
