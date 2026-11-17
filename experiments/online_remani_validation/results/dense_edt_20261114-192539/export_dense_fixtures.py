#!/usr/bin/env python3
"""Export bounded real crops and explicitly synthetic dense inputs; no worker runs."""
import os
os.environ.update(OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', PYTHONDONTWRITEBYTECODE='1')
import hashlib
import json
from pathlib import Path
import resource
import struct
import numpy as np
import rosbag

CASE = Path(__file__).resolve().parent
SOURCE = CASE.parent / 'dynamic_geometry_capture_20261114-175950' / 'occupancy_evidence.bag'
OUT = CASE / 'fixtures'


def decode(raw):
    payload = raw[1]
    dimensions, = struct.unpack_from('<I', payload, 0)
    if dimensions > 8:
        raise ValueError('Invalid MultiArray layout')
    offset = 4
    for _ in range(dimensions):
        length, = struct.unpack_from('<I', payload, offset)
        offset += 4 + length + 8
    data_offset, count = struct.unpack_from('<II', payload, offset)
    offset += 8
    if data_offset or len(payload) != offset + 8 * count:
        raise ValueError('Unexpected serialized MultiArray size/offset')
    return np.frombuffer(payload, dtype='<f8', count=count, offset=offset)


def main():
    OUT.mkdir(exist_ok=False)
    frames = []
    table = []

    def save(name, group, occupancy, resolution, minimum, metadata):
        occupancy = np.ascontiguousarray(occupancy, dtype=np.uint8)
        if not np.all((occupancy == 0) | (occupancy == 1)):
            raise ValueError('Nonbinary input')
        path = OUT / (name + '.occupancy.u8')
        occupancy.tofile(path)
        row = dict(name=name, group=group, file=path.name,
                   shape=list(occupancy.shape), min_index=list(minimum),
                   resolution=resolution, occupied=int(occupancy.sum()),
                   voxels=int(occupancy.size),
                   sha256=hashlib.sha256(path.read_bytes()).hexdigest(), **metadata)
        frames.append(row)
        table.append('%s %s %s %d %d %d %d %d %d %.17g\n' %
                     (name, group, path.name, *occupancy.shape, *minimum, resolution))

    with rosbag.Bag(str(SOURCE)) as bag:
        for index, (_, raw, received) in enumerate(bag.read_messages(
                topics=['/mppi/collision_diagnostic'], raw=True)):
            a = decode(raw)
            h = a[:40]
            n = int(h[38])
            shape = tuple(int(v) for v in h[5:8])
            if tuple(h[32:34]) != (20261112., 1.) or len(a) != 40 + 2*n:
                raise ValueError('Diagnostic schema mismatch')
            if n != int(np.prod(shape)) or h[29] != h[36] or h[4] != 1:
                raise ValueError('Require valid coherent occupancy/ESDF crop')
            codes = a[40+n:].astype(np.uint8)
            if not np.array_equal(codes, a[40+n:]) or codes.max() > 3:
                raise ValueError('Invalid occupancy codes')
            occupancy = ((codes & 2) != 0).reshape(shape)
            save('real_%02d' % index, 'real_crop', occupancy, float(h[11]), (0, 0, 0),
                 dict(source_message_sha256=hashlib.sha256(raw[1]).hexdigest(),
                      source_received_ros=received.to_sec(), acquisition_ros=float(h[37]),
                      fusion_ros=float(h[3]), origin_edge_world=h[8:11].tolist(),
                      epoch=int(h[34]), occupancy_sequence=int(h[29]), esdf_sequence=int(h[36]),
                      worker_semantics='Independent closed ROI rebased to index zero; no cross-frame changes inferred.'))
    if len(frames) != 16:
        raise ValueError('Expected exactly 16 recorded diagnostic frames')

    shape = (200, 184, 60)  # 2,208,000 voxels, deliberately NOT a recorded job.
    empty = np.zeros(shape, dtype=np.uint8)
    save('synthetic_empty', 'synthetic', empty, .05, (7, 11, 3),
         dict(construction='All free; uncapped no-obstacle sentinel behavior.'))
    rng = np.random.RandomState(20261114)
    dense = (rng.randint(0, 8, size=shape, dtype=np.uint8) == 0).astype(np.uint8)
    save('synthetic_dense', 'synthetic', dense, .05, (7, 11, 3),
         dict(construction='Seeded independent 1/8 occupied binary cells; RandomState seed 20261114.'))
    del dense
    thick = empty
    thick[0:65, 0:70, 0:45] = 1
    thick[90:180, 70:165, 10:55] = 1
    thick[:, :, :5] = 1
    thick[110:118, 100:108, 20:28] = 0
    save('synthetic_thick', 'synthetic', thick, .05, (7, 11, 3),
         dict(construction='Two thick solid blocks, boundary-touching floor, internal cavity; forces negative EDT fallback.'))

    metadata = dict(source_bag=str(SOURCE.relative_to(CASE.parent)),
        source_bag_bytes=SOURCE.stat().st_size,
        occupancy_input='Diagnostic bit1 inflated occupancy, x-major z-contiguous uint8.',
        distance_cap=0.0, changed_cells=[], trace_timing=True,
        real_scope='16 independent 403440-cell crops, not original full worker inputs or every fused generation.',
        boundary_scope='Closed-crop distances are compared before/after, not against global ESDF crop edges.',
        synthetic_scope='2.208M-cell generated inputs are scale/boundary supplements, not measured real jobs.',
        extraction_peak_rss_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.,
        frames=frames)
    (OUT / 'frames.tsv').write_text(''.join(table))
    (OUT / 'metadata.json').write_text(json.dumps(metadata, indent=2) + '\n')
    print(json.dumps(dict(frames=len(frames), fixture_bytes=sum(x['voxels'] for x in frames),
                         peak_rss_mib=metadata['extraction_peak_rss_mib'])))


if __name__ == '__main__':
    main()
