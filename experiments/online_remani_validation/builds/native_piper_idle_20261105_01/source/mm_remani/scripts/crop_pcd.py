#!/usr/bin/env python3
"""Crop robot body from binary PCD. Same format logic as shift_pcd.py."""
import sys
import numpy as np

def crop_pcd(in_path, out_path, min_x, max_x, min_y, max_y, min_z, max_z,
             ground_z=None):
    with open(in_path, 'rb') as f:
        raw = f.read()

    marker = b'DATA binary\n'
    header_end = raw.find(marker)
    header_end += len(marker)
    header = raw[:header_end]
    data = raw[header_end:]

    arr = np.frombuffer(data, dtype=np.float32).copy().reshape(-1, 8)

    n_before = len(arr)

    # Keep points OUTSIDE the box (negative crop = remove body, keep environment)
    mask = (arr[:, 0] < min_x) | (arr[:, 0] > max_x) | \
           (arr[:, 1] < min_y) | (arr[:, 1] > max_y) | \
           (arr[:, 2] < min_z) | (arr[:, 2] > max_z)

    # Optional: remove ground (points below ground_z in map frame)
    if ground_z is not None:
        mask = mask & (arr[:, 2] > ground_z)
        n_ground = (~(arr[:, 2] > ground_z)).sum()
    else:
        n_ground = 0

    arr = arr[mask]

    n_after = len(arr)

    # Update POINTS in header
    header_str = header.decode()
    for line in header_str.split('\n'):
        if 'POINTS' in line:
            header_str = header_str.replace(line, f'POINTS {n_after}')
            break
    # Also update WIDTH
    for line in header_str.split('\n'):
        if 'WIDTH' in line:
            header_str = header_str.replace(line, f'WIDTH {n_after}')
            break

    with open(out_path, 'wb') as f:
        f.write(header_str.encode())
        f.write(arr.tobytes())

    print(f"Removed {n_before - n_after} points (body + {n_ground} ground), kept {n_after} points")
    print(f"Written to {out_path}")

if __name__ == "__main__":
    if len(sys.argv) < 8:
        print("Usage: crop_pcd.py in.pcd out.pcd min_x max_x min_y max_y min_z max_z [ground_z]")
        print("Example: crop_pcd.py scans.pcd scans_clean.pcd -0.8 0.15 -0.35 0.35 -0.01 0.7 -0.15")
        sys.exit(1)
    args = [float(x) for x in sys.argv[3:]]
    crop_pcd(sys.argv[1], sys.argv[2], *args[:6], ground_z=args[6] if len(args) > 6 else None)
