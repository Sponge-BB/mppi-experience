"""Bounded, read-only prediction wire; rendering runs outside the controller."""
import numpy as np

MAGIC = 20261112.
MAX_CANDIDATES = 32
MAX_STEPS = 64


def decode_prediction(data):
    wire = np.asarray(data, dtype=np.float64)
    if (wire.ndim != 1 or len(wire) < 8 or len(wire) > 8192 or
            not np.isfinite(wire).all() or tuple(wire[:2]) != (MAGIC, 1.)):
        raise ValueError('invalid prediction visualization packet')
    stamp, dt, status, count, steps, selected_steps = wire[2:8]
    integers = (status, count, steps, selected_steps)
    if (any(v != int(v) for v in integers) or stamp <= 0 or not 0 < dt <= 1 or
            status not in (0, 1, 2) or not 0 <= count <= MAX_CANDIDATES or
            not 1 <= steps <= MAX_STEPS or not 0 <= selected_steps <= MAX_STEPS):
        raise ValueError('invalid prediction visualization dimensions')
    count, steps, selected_steps = int(count), int(steps), int(selected_steps)
    end = 8 + count + count*steps*3
    if len(wire) != end + selected_steps*17:
        raise ValueError('incomplete prediction visualization packet')
    flags = wire[8:8+count]
    if not np.isin(flags, (0., 1.)).all():
        raise ValueError('invalid prediction validity flags')
    return dict(stamp=float(stamp), dt=float(dt), status=int(status),
                valid=flags.astype(bool),
                candidates=wire[8+count:end].reshape((count, steps, 3)),
                selected=wire[end:].reshape((selected_steps, 17)))


def encode_prediction(stamp, dt, status, rollouts, selected=None):
    selected = np.empty((0, 17)) if selected is None else np.asarray(selected)
    if rollouts is None:
        if not len(selected):
            return None
        candidates = np.empty((0, len(selected), 3)); valid = np.empty(0)
    else:
        candidates, valid = rollouts
    wire = np.concatenate((
        [MAGIC, 1., stamp, dt, status, len(candidates), candidates.shape[1], len(selected)],
        np.asarray(valid, dtype=float).ravel(), candidates.ravel(), selected.ravel()))
    decode_prediction(wire)
    return wire
