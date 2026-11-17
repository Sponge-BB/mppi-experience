"""Wire-format checks for fast PointCloud2 input decoding."""
from array import array
import struct
from types import SimpleNamespace as NS
import unittest

import numpy as np

from pointcloud_numpy import read_points_numpy


def field(name, offset, datatype=7, count=1):
    return NS(name=name, offset=offset, datatype=datatype, count=count)


def cloud(data, fields=None, width=1, height=1, point_step=16, row_step=None, big=False):
    return NS(data=data, fields=fields or [field('x', 0), field('y', 4), field('z', 8)],
        width=width, height=height, point_step=point_step,
        row_step=width*point_step if row_step is None else row_step, is_bigendian=big)


class PointCloudInput(unittest.TestCase):
    def test_standard_interleaved_layout_keeps_nan_and_requested_order(self):
        data=struct.pack('<8f', 1., 2., 3., 40., 4., float('nan'), 6., 70.)
        msg=cloud(data, width=2)
        result=read_points_numpy(msg, ('z', 'x', 'y'))
        self.assertEqual(result.shape, (2, 3))
        np.testing.assert_allclose(result[0], [3., 1., 2.])
        self.assertTrue(np.isnan(result[1, 2]))
        self.assertTrue(result.flags.owndata)

    def test_organized_cloud_row_padding_and_reordered_field_offsets(self):
        data=bytearray([0xD3]*80)
        xyz=[]
        for row in range(2):
            for col in range(2):
                k=row*2+col
                struct.pack_into('<4f', data, row*40+col*16, 10.+k, 20.+k, 30.+k, 99.)
                xyz.append([30.+k, 10.+k, 20.+k])
        fields=[field('z', 4), field('x', 8), field('intensity', 12), field('y', 0)]
        result=read_points_numpy(cloud(data, fields, 2, 2, row_step=40), ('x', 'y', 'z'))
        np.testing.assert_array_equal(result, xyz)

    def test_big_endian_mixed_uint32_track_identifiers_preserve_precision(self):
        data=struct.pack('>fIfI', 1.25, 0xFFFFFFFF, -2.5, 0x80000001)
        fields=[field('x', 0), field('track_id', 4, 6)]
        result=read_points_numpy(cloud(data, fields, 2, point_step=8, big=True), ('track_id', 'x'))
        self.assertEqual(result.dtype, np.dtype('float64'))
        self.assertEqual(int(result[0, 0]), 0xFFFFFFFF)
        self.assertEqual(int(result[1, 0]), 0x80000001)
        np.testing.assert_array_equal(result[:, 1], [1.25, -2.5])

    def test_supports_array_byte_buffer_and_all_ros_scalar_datatypes(self):
        formats=[('i8', 'b', 1, -7), ('u8', 'B', 2, 250), ('i16', 'h', 3, -1234),
                 ('u16', 'H', 4, 65000), ('i32', 'i', 5, -123456789),
                 ('u32', 'I', 6, 4000000001), ('f32', 'f', 7, 1.5), ('f64', 'd', 8, .125)]
        data=bytearray(); fields=[]
        for name, fmt, datatype, value in formats:
            fields.append(field(name, len(data), datatype))
            data.extend(struct.pack('<'+fmt, value))
        msg=cloud(array('B', data), fields, point_step=len(data))
        result=read_points_numpy(msg, tuple(f[0] for f in formats))
        np.testing.assert_array_equal(result[0], [f[3] for f in formats])

    def test_rejects_truncation_extra_bytes_and_invalid_row_stride(self):
        for data, row_step, reason in [(bytes(15), 16, 'size_mismatch'),
                                       (bytes(17), 16, 'size_mismatch'),
                                       (bytes(12), 12, 'row_step')]:
            with self.assertRaisesRegex(ValueError, reason):
                read_points_numpy(cloud(data, row_step=row_step), ('x',))
        with self.assertRaisesRegex(ValueError, 'size_mismatch'):
            read_points_numpy(cloud(bytes(72), width=2, height=2, row_step=40), ('x',))

    def test_rejects_duplicate_missing_overlapping_and_out_of_bounds_fields(self):
        cases=[([field('x', 0), field('x', 4)], 'duplicate'),
               ([field('y', 0)], 'missing'),
               ([field('x', 14)], 'exceeds'),
               ([field('x', 0), field('y', 2)], 'overlapping'),
               ([field('x', 0, 99)], 'datatype'),
               ([field('x', -1)], 'offset'),
               ([field('x', 0, 7, 2)], 'count_one')]
        for fields, reason in cases:
            with self.subTest(reason=reason), self.assertRaisesRegex(ValueError, reason):
                read_points_numpy(cloud(bytes(16), fields), ('x',))
        with self.assertRaisesRegex(ValueError, 'requested_fields'):
            read_points_numpy(cloud(bytes(16)), ('x', 'x'))

    def test_empty_cloud_retains_shape_without_fabricating_points(self):
        result=read_points_numpy(cloud(b'', width=0), ('x', 'y', 'z'))
        self.assertEqual(result.shape, (0, 3))


if __name__ == '__main__':
    unittest.main()
