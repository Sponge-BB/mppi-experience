"""Small diagnostic bag only; raw ROS arrays remain NumPy buffer views."""
from pathlib import Path
import datetime
import hashlib
import json
import struct
import numpy as np
import rosbag
from scipy.ndimage import distance_transform_edt

HERE = Path(__file__).resolve().parent
CASE = HERE.parent


def decode_array(raw):
    payload = raw[1]
    dims, = struct.unpack_from('<I', payload, 0)
    offset = 4
    for _ in range(dims):
        count, = struct.unpack_from('<I', payload, offset)
        offset += 4 + count + 8
    data_offset, count = struct.unpack_from('<II', payload, offset)
    offset += 8
    assert data_offset == 0
    assert len(payload) == offset + 8 * count
    return np.frombuffer(payload, dtype='<f8', count=count, offset=offset)


def local(stamp):
    return datetime.datetime.fromtimestamp(stamp).isoformat(timespec='milliseconds')


def analyze():
    frames = []
    last = None
    with rosbag.Bag(str(CASE / 'occupancy_evidence.bag')) as bag:
        for i, (_, raw, receipt) in enumerate(bag.read_messages(raw=True)):
            a = decode_array(raw)
            h = a[:40]
            n = int(h[38])
            shape = h[5:8].astype(int)
            assert h[32] == 20261112 and h[33] == 1
            assert n == np.prod(shape) and a.size == 40 + 2 * n
            esdf = a[40:40+n]
            codes_wire = a[40+n:]
            codes = codes_wire.astype(np.uint8)
            assert np.array_equal(codes, codes_wire)
            assert codes.max() <= 3
            hist = np.bincount(codes, minlength=4)
            inflated = (codes & 2) != 0
            raw_occ = (codes & 1) != 0
            # Current recorded field saturates at 0.6 m. Crop-edge cells cannot
            # independently reconstruct outside obstacles, so exclude that rim.
            distance_cap = .6
            occ_grid = inflated.reshape(shape)
            expected = distance_transform_edt(~occ_grid) * h[11]
            expected -= np.maximum(distance_transform_edt(occ_grid) * h[11] - h[11], 0.)
            np.clip(expected, -distance_cap, distance_cap, out=expected)
            rim = int(np.ceil(distance_cap/h[11])) + 1
            core = tuple(slice(rim, int(s)-rim) for s in shape)
            delta = np.abs(expected[core] - esdf.reshape(shape)[core])
            by_code = {}
            for code in range(4):
                d = esdf[codes == code]
                by_code[str(code)] = dict(count=int(d.size), min=None if not d.size else float(d.min()),
                                          max=None if not d.size else float(d.max()))
            idx = np.argwhere(inflated.reshape(shape))
            points = h[8:11] + (idx + .5) * h[11]
            frame = dict(index=i, receipt=receipt.to_sec(), receipt_local=local(receipt.to_sec()),
                acquisition=float(h[37]), acquisition_local=local(h[37]), fusion=float(h[3]), fusion_local=local(h[3]),
                receipt_minus_acquisition_ms=1000*(receipt.to_sec()-h[37]), map_age_at_acquisition_ms=1000*(h[37]-h[3]),
                joint_age_at_acquisition_ms=1000*(h[37]-h[24]), evidence_copy_ms=float(h[39])*1000,
                header=h.tolist(), shape=shape.tolist(), origin_edge=h[8:11].tolist(), resolution=float(h[11]),
                field_version=int(h[14]), epoch=int(h[34]), distance_revision=int(h[35]),
                occupancy_sequence=int(h[29]), esdf_sequence=int(h[36]), lidar_accepted_sequence=int(h[30]),
                lidar_fused_sequence=int(h[31]), same_generation=bool(h[29] == h[36]), valid=bool(h[4]),
                ground_valid=bool(h[22]), self_collision_valid=bool(h[23]), body_sphere_count=int(h[12]),
                cells=n, finite_esdf=bool(np.isfinite(esdf).all()), occupancy_code_counts=hist.tolist(),
                occupied_inflated_positive_esdf_count=int(np.count_nonzero(esdf[inflated] > 1e-9)),
                unoccupied_negative_esdf_count=int(np.count_nonzero(esdf[~inflated] < -1e-9)),
                distance_by_occupancy_code=by_code,
                independent_edt_core_check=dict(assumed_distance_cap=distance_cap, excluded_rim_cells=rim,
                    checked_cells=int(delta.size), max_abs_difference=float(delta.max()),
                    difference_over_1e_8_count=int(np.count_nonzero(delta>1e-8))),
                occupied_center_bounds=None if not points.size else [points.min(axis=0).tolist(), points.max(axis=0).tolist()],
                occupied_height_counts=np.count_nonzero(inflated.reshape(shape),axis=(0,1)).tolist())
            if last is not None and np.array_equal(last['origin'],h[8:11]) and np.array_equal(last['shape'],shape):
                changed = codes != last['codes']
                frame['change_from_previous_same_crop'] = dict(changed_voxels=int(np.count_nonzero(changed)),
                    raw_added=int(np.count_nonzero(raw_occ & ~((last['codes'] & 1) != 0))),
                    raw_removed=int(np.count_nonzero(~raw_occ & ((last['codes'] & 1) != 0))))
            else:
                frame['change_from_previous_same_crop'] = None
            last = dict(origin=h[8:11].copy(),shape=shape.copy(),codes=codes)
            frames.append(frame)
    report = dict(bag='occupancy_evidence.bag', bag_bytes=(CASE/'occupancy_evidence.bag').stat().st_size,
        frame_count=len(frames), all_same_generation=all(x['same_generation'] for x in frames),
        all_finite_esdf=all(x['finite_esdf'] for x in frames),
        schema='header40 + N ESDF float64 + N occupancy-code float64; code bit0 raw, bit1 inflated; x-major z-contiguous; world voxel-edge origin',
        interpretation=['Exporter obtains occupancy and distance within one immutable Complete scope; diagnostic requires identical occupancy and ESDF observation sequences.',
        'Raw/inflated occupancy is merged native map occupancy; codes do not identify static versus live source.',
        'Diagnostic header stores body sphere count and joint timestamp but omits sphere positions and odometry; body collision needs separately aligned body and odometry.',
        'No large dynamic_capture.bag was opened by this script.'], frames=frames)
    (HERE/'occupancy_metadata.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(frame_count=len(frames),first=frames[0]['acquisition_local'],last=frames[-1]['acquisition_local'],
        all_same_generation=report['all_same_generation'], all_finite_esdf=report['all_finite_esdf'],
        frames=[{k:x[k] for k in ['index','field_version','occupancy_sequence','valid','map_age_at_acquisition_ms','joint_age_at_acquisition_ms','occupancy_code_counts','occupied_inflated_positive_esdf_count','unoccupied_negative_esdf_count']} for x in frames]),indent=2))


if __name__ == '__main__':
    analyze()
