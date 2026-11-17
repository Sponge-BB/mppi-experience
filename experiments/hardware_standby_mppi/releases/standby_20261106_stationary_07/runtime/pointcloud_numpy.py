"""Vectorized PointCloud2 decoding without ROS imports or point filtering.

Metadata and all point/row bounds are checked before viewing the byte buffer.
The returned numeric matrix owns its storage, follows requested field order,
retains NaNs, and preserves UINT32 track identifiers exactly when mixed with
floating-point coordinates (NumPy promotes that combination to float64).
"""
import operator
import numpy as np


_FIELD_TYPES = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2',
                5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}


def _integer(value, name, minimum=0):
    if isinstance(value, (bool, np.bool_)):
        raise ValueError('invalid_'+name)
    try:
        value = operator.index(value)
    except TypeError:
        raise ValueError('invalid_'+name)
    if value < minimum:
        raise ValueError('invalid_'+name)
    return value


def read_points_numpy(message, field_names):
    """Decode scalar fields as N x K in row-major order, honoring wire padding.

    This function does not infer free space, skip points, or remove NaNs.
    Unknown datatypes, duplicate field names, invalid offsets, nonscalar
    requested fields, overlapping fields and truncated buffers are rejected.
    """
    names = tuple(field_names)
    if not names or any(not isinstance(n, str) or not n for n in names) or len(set(names)) != len(names):
        raise ValueError('invalid_requested_fields')
    width = _integer(message.width, 'width')
    height = _integer(message.height, 'height')
    point_step = _integer(message.point_step, 'point_step', 1)
    row_step = _integer(message.row_step, 'row_step')
    if height == 0 and width != 0:
        raise ValueError('invalid_height')
    if row_step < width*point_step:
        raise ValueError('row_step_too_small')
    try:
        buffer = memoryview(message.data)
    except TypeError:
        raise ValueError('point_data_has_no_byte_buffer')
    if not buffer.c_contiguous:
        raise ValueError('point_data_not_contiguous')
    buffer = buffer.cast('B')
    required = height*row_step
    if buffer.nbytes != required:
        raise ValueError('point_data_size_mismatch')
    endian = '>' if message.is_bigendian else '<'
    fields = {}; intervals = []
    for field in message.fields:
        name = field.name
        if not isinstance(name, str) or not name or name in fields:
            raise ValueError('duplicate_or_invalid_field_name')
        datatype = _integer(field.datatype, 'field_datatype', 1)
        if datatype not in _FIELD_TYPES:
            raise ValueError('unsupported_field_datatype')
        dtype = np.dtype(endian+_FIELD_TYPES[datatype])
        count = _integer(field.count, 'field_count', 1)
        offset = _integer(field.offset, 'field_offset')
        end = offset+dtype.itemsize*count
        if end > point_step:
            raise ValueError('field_exceeds_point_step')
        if any(offset < other_end and other_start < end for other_start, other_end in intervals):
            raise ValueError('overlapping_point_fields')
        intervals.append((offset, end))
        fields[name] = (dtype, offset, count)
    if any(name not in fields for name in names):
        raise ValueError('requested_field_missing')
    if any(fields[name][2] != 1 for name in names):
        raise ValueError('requested_field_must_have_count_one')
    dtype = np.dtype(dict(names=names, formats=[fields[n][0] for n in names],
                          offsets=[fields[n][1] for n in names], itemsize=point_step))
    if not width or not height:
        return np.empty((0, len(names)), dtype=np.result_type(*[fields[n][0] for n in names]))
    if row_step == width*point_step:
        records = np.frombuffer(buffer, dtype=dtype, count=width*height)
    else:
        records = np.ndarray(shape=(height, width), dtype=dtype, buffer=buffer,
                             strides=(row_step, point_step))
    # Each field is a typed, strided view, avoiding Python work per point.
    return np.column_stack([records[name].reshape(-1) for name in names])
