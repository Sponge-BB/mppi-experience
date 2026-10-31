"""Opt-in, read-only state-chain records; no ROS imports or control authority.

Message identity is over the exact ROS1 serialization, not a timestamp or an
approximate state match. Numeric validity describes representation only; it is
never an authorization or a claim that a state/command is safe.
"""
import copy
import io
import json
import math
from numbers import Integral, Real
import os
import sys
import threading


IDENTITY_ALGORITHM = 'fnv1a64_ros1_serialized_standard'
FNV1A64_OFFSET = 14695981039346656037
FNV1A64_PRIME = 1099511628211


def capture_enabled():
    return os.environ.get('STATE_CONTRACT_CAPTURE') == '1'


def fnv1a64(data):
    """Standard 64-bit FNV-1a, emitted as a lossless decimal string."""
    value = FNV1A64_OFFSET
    for byte in data:
        value = ((value ^ byte) * FNV1A64_PRIME) & 0xffffffffffffffff
    return str(value)


def sanitize_nonfinite(value, path='$'):
    """Return JSON values and the exact paths replaced with null.

    Arrays with ``tolist`` are supported without importing numpy. Missing
    values remain missing/null, and are not confused with finite zeroes.
    """
    if value is None or isinstance(value, (str, bool)):
        return value, []
    if isinstance(value, Integral):
        return int(value), []
    if isinstance(value, Real):
        number = float(value)
        return (number, []) if math.isfinite(number) else (None, [path])
    if isinstance(value, dict):
        clean = {}; invalid = []
        for key, item in value.items():
            cleaned, paths = sanitize_nonfinite(item, path + '.' + str(key))
            clean[key] = cleaned; invalid.extend(paths)
        return clean, invalid
    if isinstance(value, (list, tuple)):
        clean = []; invalid = []
        for index, item in enumerate(value):
            cleaned, paths = sanitize_nonfinite(item, '%s[%d]' % (path, index))
            clean.append(cleaned); invalid.extend(paths)
        return clean, invalid
    if hasattr(value, 'tolist'):
        return sanitize_nonfinite(value.tolist(), path)
    raise TypeError('unsupported state-contract value: ' + type(value).__name__)


def record_with_validity(record):
    clean, invalid = sanitize_nonfinite(record)
    clean['numeric_validity'] = {'all_finite': not invalid, 'nonfinite_paths': invalid}
    return clean


def odom_identity(message):
    """Extract identity and raw measured pose/body twist from one ROS1 message."""
    stream = io.BytesIO()
    message.serialize(stream)
    raw = stream.getvalue()
    header = message.header; stamp = header.stamp
    p = message.pose.pose.position; q = message.pose.pose.orientation
    linear = message.twist.twist.linear; angular = message.twist.twist.angular
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))
    identity = {
        'algorithm': IDENTITY_ALGORITHM, 'message_hash': fnv1a64(raw),
        'serialized_bytes': len(raw), 'header_seq': int(header.seq),
        'stamp_sec': int(stamp.secs), 'stamp_nsec': int(stamp.nsecs),
        'frame_id': header.frame_id, 'child_frame_id': message.child_frame_id,
        'position': [p.x, p.y, p.z], 'quaternion': [q.x, q.y, q.z, q.w],
        'yaw': yaw, 'body_twist_linear': [linear.x, linear.y, linear.z],
        'body_twist_angular': [angular.x, angular.y, angular.z],
        'state': [p.x, p.y, yaw, linear.x, angular.z],
    }
    clean, invalid = sanitize_nonfinite(identity)
    clean['validity'] = {'serialized': True, 'all_numeric_finite': not invalid,
                         'nonfinite_paths': invalid,
                         'state_finite': all(math.isfinite(float(v)) for v in identity['state'])}
    return clean


def identity_or_error(message):
    """A diagnostic failure must not replace or suppress measured control state."""
    try:
        return odom_identity(message)
    except Exception as exc:
        return {'algorithm': IDENTITY_ALGORITHM, 'message_hash': None,
                'capture_error': type(exc).__name__ + ': ' + str(exc),
                'validity': {'serialized': False, 'all_numeric_finite': False,
                             'state_finite': False}}


def copy_state_and_identity(state, identity):
    """Call while holding the same lock used by the odometry callback."""
    return (None if state is None else state.copy()), copy.deepcopy(identity)


class JsonlCapture:
    """Optional diagnostic sink: I/O failures are explicit and never gate motion."""
    def __init__(self, path, enabled=None):
        self.enabled = capture_enabled() if enabled is None else bool(enabled)
        self.path = path; self.stream = None; self.lock = threading.Lock()
        if self.enabled:
            try:
                self.stream = path.open('w', buffering=1)
            except Exception as exc:
                self._error('open', exc)

    def _error(self, operation, exc):
        # stderr stays independent of ROS publishers and command authority.
        try:
            print('STATE_CONTRACT_CAPTURE_ERROR ' + json.dumps({
                'operation': operation, 'path': str(self.path),
                'error': type(exc).__name__ + ': ' + str(exc)}, allow_nan=False),
                file=sys.stderr, flush=True)
        except Exception:
            # Even failure of the last diagnostic channel cannot gate control.
            pass

    def write(self, record):
        if not self.enabled or self.stream is None:
            return False
        with self.lock:
            try:
                if self.stream.closed:
                    return False
                self.stream.write(json.dumps(record_with_validity(record),
                                             separators=(',', ':'), allow_nan=False) + '\n')
                return True
            except Exception as exc:
                self._error('write', exc)
                return False

    def close(self):
        if self.stream is not None:
            with self.lock:
                try:
                    if not self.stream.closed:
                        self.stream.close()
                except Exception as exc:
                    self._error('close', exc)
