#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import struct
import threading
import time

import rospy
import serial
from geometry_msgs.msg import Twist


class CmdVelToUartNode:
    TX_HEADER = 0xAA
    TX_TAIL = 0x55
    RX_HEADER = 0xAB
    RX_TAIL = 0xBA
    TX_FRAME_FORMAT = '<B3fB'
    RX_PAYLOAD_FORMAT = '<3f'
    RX_FRAME_SIZE = 14
    ZERO_VELOCITY = (0.0, 0.0, 0.0)

    def __init__(self):
        rospy.init_node('cmd_vel_to_uart_node', anonymous=True)

        self.port_name = rospy.get_param('~port', '/dev/ttyUSB_base')
        self.baud_rate = int(rospy.get_param('~baud', 115200))
        self.read_timeout = max(0.001, float(rospy.get_param('~read_timeout', 0.02)))
        self.write_timeout = max(0.001, float(rospy.get_param('~write_timeout', 0.05)))
        self.send_rate = max(1.0, float(rospy.get_param('~send_rate', 50.0)))
        self.command_timeout = max(0.05, float(rospy.get_param('~command_timeout', 0.30)))
        self.feedback_publish_rate = max(
            1.0, float(rospy.get_param('~feedback_publish_rate', 50.0)))
        self.read_chunk_size = max(
            self.RX_FRAME_SIZE, int(rospy.get_param('~read_chunk_size', 64)))
        self.shutdown_zero_frames = max(
            1, int(rospy.get_param('~shutdown_zero_frames', 3)))

        self._state_lock = threading.Lock()
        self._serial_write_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._shutdown_started = False
        self._latest_command = self.ZERO_VELOCITY
        self._latest_command_monotonic = None
        self._latest_feedback = self.ZERO_VELOCITY
        self._feedback_seq = 0
        self._published_feedback_seq = 0
        self._rx_buffer = bytearray()
        self.ser = None
        self.read_thread = None
        self.send_timer = None
        self.feedback_timer = None

        try:
            self.ser = serial.Serial(
                self.port_name,
                self.baud_rate,
                timeout=self.read_timeout,
                write_timeout=self.write_timeout)
        except (serial.SerialException, ValueError, OSError) as exc:
            rospy.logerr(
                "无法打开串口 %s ! 请检查权限或接线。错误信息: %s",
                self.port_name, exc)
            rospy.signal_shutdown("串口打开失败")
            return

        self.vel_pub = rospy.Publisher('/my_car_vel', Twist, queue_size=1)
        self.cmd_sub = rospy.Subscriber(
            '/cmd_vel', Twist, self.cmd_vel_callback,
            queue_size=1, tcp_nodelay=True)

        rospy.on_shutdown(self.shutdown_hook)

        self.read_thread = threading.Thread(
            target=self.serial_read_loop, name='base_uart_rx', daemon=True)
        self.read_thread.start()
        self.send_timer = rospy.Timer(
            rospy.Duration(1.0 / self.send_rate), self.send_timer_callback,
            reset=True)
        self.feedback_timer = rospy.Timer(
            rospy.Duration(1.0 / self.feedback_publish_rate),
            self.feedback_timer_callback, reset=True)

        rospy.loginfo(
            "底盘串口已打开: %s @ %d; tx=%.1fHz, feedback<=%.1fHz, "
            "cmd_timeout=%.3fs, read_timeout=%.3fs, write_timeout=%.3fs",
            self.port_name, self.baud_rate, self.send_rate,
            self.feedback_publish_rate, self.command_timeout,
            self.read_timeout, self.write_timeout)

    def cmd_vel_callback(self, msg):
        """Cache only the newest command; the fixed-rate sender owns UART writes."""
        command = (msg.linear.x, msg.linear.y, msg.angular.z)
        if not all(math.isfinite(value) for value in command):
            rospy.logerr_throttle(
                1.0, "收到非有限 /cmd_vel，按零速度处理: %s", command)
            command = self.ZERO_VELOCITY

        with self._state_lock:
            self._latest_command = command
            self._latest_command_monotonic = time.monotonic()

    def _command_for_tx(self, now_monotonic=None):
        if now_monotonic is None:
            now_monotonic = time.monotonic()
        with self._state_lock:
            command = self._latest_command
            command_stamp = self._latest_command_monotonic

        if command_stamp is None:
            return self.ZERO_VELOCITY, True
        if now_monotonic - command_stamp > self.command_timeout:
            return self.ZERO_VELOCITY, True
        return command, False

    def _write_velocity_frame(self, velocity, log_errors=True):
        frame = struct.pack(
            self.TX_FRAME_FORMAT,
            self.TX_HEADER,
            float(velocity[0]), float(velocity[1]), float(velocity[2]),
            self.TX_TAIL)
        try:
            with self._serial_write_lock:
                if self.ser is None or not self.ser.is_open:
                    return False
                written = self.ser.write(frame)
            if written != len(frame):
                if log_errors:
                    rospy.logerr_throttle(
                        1.0, "串口短写: %d/%d bytes", written, len(frame))
                return False
            return True
        except (serial.SerialException, serial.SerialTimeoutException, OSError) as exc:
            if log_errors:
                rospy.logerr_throttle(1.0, "串口发送失败: %s", exc)
            return False
        except Exception as exc:
            if log_errors:
                rospy.logerr_throttle(1.0, "串口发送异常: %s", exc)
            return False

    def send_timer_callback(self, _event):
        if self._stop_event.is_set():
            return
        command, timed_out = self._command_for_tx()
        if timed_out and self._latest_command_monotonic is not None:
            rospy.logwarn_throttle(
                1.0, "/cmd_vel 超过 %.3fs 未更新，持续发送零速度",
                self.command_timeout)
        self._write_velocity_frame(command)

    def _consume_rx_bytes(self, data):
        """Append bytes and return every complete, valid AB+3float+BA frame."""
        if data:
            self._rx_buffer.extend(data)
        decoded = []

        while self._rx_buffer:
            header_index = self._rx_buffer.find(bytes((self.RX_HEADER,)))
            if header_index < 0:
                self._rx_buffer.clear()
                break
            if header_index > 0:
                del self._rx_buffer[:header_index]
            if len(self._rx_buffer) < self.RX_FRAME_SIZE:
                break

            if self._rx_buffer[self.RX_FRAME_SIZE - 1] != self.RX_TAIL:
                # Drop only the false header and search again. Clearing the whole
                # serial input would also discard a valid frame already buffered.
                del self._rx_buffer[0]
                continue

            payload = self._rx_buffer[1:self.RX_FRAME_SIZE - 1]
            decoded.append(struct.unpack(self.RX_PAYLOAD_FORMAT, payload))
            del self._rx_buffer[:self.RX_FRAME_SIZE]

        return decoded

    def serial_read_loop(self):
        """Blocking short-timeout UART reader with byte-stream resynchronization."""
        while not self._stop_event.is_set() and not rospy.is_shutdown():
            try:
                # read() blocks until the configured timeout or a chunk is ready,
                # eliminating the previous in_waiting busy-poll loop.
                data = self.ser.read(self.read_chunk_size)
                if not data:
                    continue
                frames = self._consume_rx_bytes(data)
                for feedback in frames:
                    if not all(math.isfinite(value) for value in feedback):
                        rospy.logwarn_throttle(
                            1.0, "忽略包含非有限数值的底盘反馈: %s", feedback)
                        continue
                    with self._state_lock:
                        self._latest_feedback = feedback
                        self._feedback_seq += 1
            except (serial.SerialException, OSError) as exc:
                if not self._stop_event.is_set():
                    rospy.logwarn_throttle(1.0, "串口读取失败: %s", exc)
                    self._stop_event.wait(0.05)
            except Exception as exc:
                if not self._stop_event.is_set():
                    rospy.logwarn_throttle(1.0, "串口读取异常: %s", exc)
                    self._stop_event.wait(0.05)

    def feedback_timer_callback(self, _event):
        if self._stop_event.is_set():
            return
        with self._state_lock:
            if self._feedback_seq == self._published_feedback_seq:
                return
            feedback = self._latest_feedback
            self._published_feedback_seq = self._feedback_seq

        msg = Twist()
        msg.linear.x = feedback[0]
        msg.linear.y = feedback[1]
        msg.angular.z = feedback[2]
        self.vel_pub.publish(msg)

    def shutdown_hook(self):
        """Stop worker activity, send repeated zero frames, then close UART."""
        if self._shutdown_started:
            return
        self._shutdown_started = True
        rospy.logwarn("底盘串口节点关闭：重复下发零速度后关闭串口")
        self._stop_event.set()

        if self.send_timer is not None:
            self.send_timer.shutdown()
        if self.feedback_timer is not None:
            self.feedback_timer.shutdown()

        if self.ser is not None and self.ser.is_open:
            for _ in range(self.shutdown_zero_frames):
                self._write_velocity_frame(self.ZERO_VELOCITY, log_errors=False)
                time.sleep(0.01)
            try:
                self.ser.close()
            except Exception as exc:
                rospy.logerr("关闭底盘串口失败: %s", exc)

        if (self.read_thread is not None and self.read_thread.is_alive() and
                self.read_thread is not threading.current_thread()):
            self.read_thread.join(timeout=max(0.2, 2.0 * self.read_timeout))
        rospy.loginfo("底盘串口已安全关闭")


if __name__ == '__main__':
    try:
        node = CmdVelToUartNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
