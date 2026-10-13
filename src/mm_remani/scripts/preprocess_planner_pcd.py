#!/usr/bin/env python3
"""Build a small, deterministic XYZ PCD for the REMANI planner.

The filtering order intentionally matches ``publish_shifted_pcd.py``:

  initial rigid transform -> world-height filter -> optional sparse-outlier
  filter -> minimum-points-per-voxel filter -> one point per voxel

The selected points are transformed back to the *map-input* coordinates before
they are written.  Consequently the generated PCD keeps the normal launch
semantics: use the same ``robot_map_*`` and ``map_{roll,pitch,yaw}`` values at
runtime, set ``input_is_preprocessed:=true``, and the published world points
will match the legacy pipeline.  Runtime map corrections then need only a rigid
transform of this small XYZ cloud.
"""

import argparse
import hashlib
import os
import resource
import sys
import tempfile
import time

import numpy as np


DATA_MARKER = b"DATA binary\n"


def parse_binary_pcd(path):
    """Return an mmap plus basic metadata for a float32 binary PCD."""
    header_lines = []
    with open(path, "rb") as stream:
        while True:
            line = stream.readline()
            if not line:
                raise RuntimeError("PCD header ended before 'DATA binary'")
            header_lines.append(line)
            if line == DATA_MARKER:
                break
            if len(header_lines) > 256:
                raise RuntimeError("PCD header is unexpectedly long")
        data_offset = stream.tell()

    values = {}
    for raw_line in header_lines:
        line = raw_line.decode("ascii").strip()
        if not line or line.startswith("#"):
            continue
        tokens = line.split()
        values[tokens[0].upper()] = tokens[1:]

    fields = values.get("FIELDS", [])
    if not fields:
        raise RuntimeError("PCD has no FIELDS entry")
    sizes = [int(value) for value in values.get("SIZE", [])]
    types = values.get("TYPE", [])
    counts = [int(value) for value in values.get("COUNT", ["1"] * len(fields))]
    if not (len(fields) == len(sizes) == len(types) == len(counts)):
        raise RuntimeError("PCD FIELDS/SIZE/TYPE/COUNT lengths do not match")
    if any(size != 4 for size in sizes) or any(kind != "F" for kind in types):
        raise RuntimeError("Only float32 PCD fields are supported")
    if any(count != 1 for count in counts):
        raise RuntimeError("Only scalar PCD fields are supported")
    for required in ("x", "y", "z"):
        if required not in fields:
            raise RuntimeError("PCD is missing required field '%s'" % required)

    if "POINTS" in values:
        point_count = int(values["POINTS"][0])
    else:
        point_count = int(values["WIDTH"][0]) * int(values["HEIGHT"][0])
    expected_size = data_offset + point_count * len(fields) * 4
    actual_size = os.path.getsize(path)
    if actual_size != expected_size:
        raise RuntimeError(
            "PCD byte size mismatch: header says %d points (%d bytes expected), file has %d bytes"
            % (point_count, expected_size, actual_size))

    points = np.memmap(
        path, dtype=np.float32, mode="r", offset=data_offset,
        shape=(point_count, len(fields)))
    xyz_indices = [fields.index(name) for name in ("x", "y", "z")]
    return points, fields, xyz_indices, data_offset


def initial_transform(robot_x, robot_y, robot_z, roll_deg, pitch_deg, yaw_deg):
    roll, pitch, yaw = np.radians([roll_deg, pitch_deg, yaw_deg])
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    rotation = rz @ ry @ rx
    translation = np.array([-robot_x, -robot_y, -robot_z], dtype=np.float64)
    return rotation, translation


def transform_and_height_filter(points, xyz_indices, rotation, translation,
                                ground_min, height_max, chunk_points):
    selected_chunks = []
    for start in range(0, len(points), chunk_points):
        stop = min(start + chunk_points, len(points))
        xyz = np.asarray(points[start:stop, xyz_indices], dtype=np.float32)
        # Match publish_shifted_pcd.py: float64 matrix expression assigned back
        # into a float32 point array before filtering.
        world_xyz = np.asarray(
            (rotation @ xyz.T).T + translation, dtype=np.float32)
        finite = np.isfinite(world_xyz).all(axis=1)
        height = ((world_xyz[:, 2] >= ground_min) &
                  (world_xyz[:, 2] <= height_max))
        selected = world_xyz[finite & height]
        if len(selected):
            selected_chunks.append(selected)
        print(
            "\rHeight filter: %d / %d input points scanned" % (stop, len(points)),
            end="", file=sys.stderr, flush=True)
    print(file=sys.stderr)
    if not selected_chunks:
        return np.empty((0, 3), dtype=np.float32)
    return np.concatenate(selected_chunks, axis=0)


def filter_sparse_outliers(points, radius, min_neighbors):
    """Voxel-neighborhood filter equivalent to the online implementation."""
    voxel_keys = np.floor(points[:, :3] / radius).astype(np.int32)
    unique_keys, inverse, counts = np.unique(
        voxel_keys, axis=0, return_inverse=True, return_counts=True)
    count_by_key = {
        tuple(key): int(count) for key, count in zip(unique_keys, counts)
    }
    keep_voxel = np.zeros(len(unique_keys), dtype=bool)
    neighbor_offsets = [
        (dx, dy, dz)
        for dx in (-1, 0, 1)
        for dy in (-1, 0, 1)
        for dz in (-1, 0, 1)
    ]
    for index, key in enumerate(unique_keys):
        key_tuple = tuple(key)
        neighbor_count = 0
        for offset in neighbor_offsets:
            neighbor_key = (
                key_tuple[0] + offset[0],
                key_tuple[1] + offset[1],
                key_tuple[2] + offset[2],
            )
            neighbor_count += count_by_key.get(neighbor_key, 0)
            if neighbor_count >= min_neighbors:
                keep_voxel[index] = True
                break
    return points[keep_voxel[inverse]]


def legacy_voxel_keys(points, voxel_res):
    """Preserve the legacy scalar-key behavior for an identical artifact."""
    inverse_res = 1.0 / voxel_res
    voxel_keys = (points[:, :3] * inverse_res).astype(np.int32)
    max_coord = int(inverse_res * 100)
    return (voxel_keys[:, 0].astype(np.int64) * max_coord * max_coord +
            voxel_keys[:, 1].astype(np.int64) * max_coord +
            voxel_keys[:, 2].astype(np.int64))


def density_filter_and_downsample(points, voxel_res, min_voxel_points):
    before_density = len(points)
    if min_voxel_points > 1 and len(points):
        keys = legacy_voxel_keys(points, voxel_res)
        _, inverse, counts = np.unique(keys, return_inverse=True, return_counts=True)
        points = points[counts[inverse] >= min_voxel_points]

    density_count = len(points)
    keys = legacy_voxel_keys(points, voxel_res)
    _, unique_indices = np.unique(keys, return_index=True)
    return points[unique_indices], before_density, density_count


def world_to_map_input(world_points, rotation, translation):
    map_points = (rotation.T @ (world_points - translation).T).T
    return np.ascontiguousarray(map_points, dtype=np.float32)


def xyz_pcd_header(point_count, args):
    return (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "# REMANI_PREPROCESSED_PLANNER_PCD map-input coordinates\n"
        "# PREPROCESS robot_map %.9g %.9g %.9g rpy_deg %.9g %.9g %.9g\n"
        "# FILTER world_z %.9g %.9g voxel %.9g min_voxel_points %d outlier %d %.9g %d\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        "WIDTH %d\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        "POINTS %d\n"
        "DATA binary\n"
    ) % (
        args.robot_map_x, args.robot_map_y, args.robot_map_z,
        args.map_roll, args.map_pitch, args.map_yaw,
        args.ground_min, args.height_max, args.voxel_res,
        args.min_voxel_points, int(args.outlier_filter_enable),
        args.outlier_filter_radius, args.outlier_filter_min_neighbors,
        point_count, point_count,
    )


def write_xyz_binary_pcd(path, points, args):
    output_dir = os.path.dirname(os.path.abspath(path))
    os.makedirs(output_dir, exist_ok=True)
    header = xyz_pcd_header(len(points), args).encode("ascii")
    with tempfile.NamedTemporaryFile(
            prefix=".planner_pcd_", suffix=".tmp", dir=output_dir,
            delete=False) as stream:
        temporary_path = stream.name
        try:
            stream.write(header)
            stream.write(np.ascontiguousarray(points, dtype=np.float32).tobytes())
            stream.flush()
            os.fsync(stream.fileno())
            os.chmod(temporary_path, 0o644)
        except Exception:
            os.unlink(temporary_path)
            raise
    os.replace(temporary_path, path)


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def ranges(points):
    return np.min(points, axis=0), np.max(points, axis=0)


def build_argument_parser():
    parser = argparse.ArgumentParser(
        description="Build the deterministic XYZ planner PCD used by exp0_real.")
    parser.add_argument("input", help="raw binary PCD (for example scans.pcd)")
    parser.add_argument("output", help="output XYZ binary PCD")
    parser.add_argument("--robot-map-x", type=float, default=0.0)
    parser.add_argument("--robot-map-y", type=float, default=0.0)
    parser.add_argument("--robot-map-z", type=float, default=-0.075)
    parser.add_argument("--map-roll", type=float, default=0.0, help="degrees")
    parser.add_argument("--map-pitch", type=float, default=0.0, help="degrees")
    parser.add_argument("--map-yaw", type=float, default=0.0, help="degrees")
    parser.add_argument("--ground-min", type=float, default=0.13)
    parser.add_argument("--height-max", type=float, default=1.70)
    parser.add_argument("--voxel-res", type=float, default=0.08)
    parser.add_argument("--min-voxel-points", type=int, default=4)
    parser.add_argument(
        "--outlier-filter-enable", dest="outlier_filter_enable",
        action="store_true", help="enable the sparse-outlier filter (default)")
    parser.add_argument(
        "--no-outlier-filter", dest="outlier_filter_enable",
        action="store_false", help="disable the sparse-outlier filter")
    parser.set_defaults(outlier_filter_enable=True)
    parser.add_argument("--outlier-filter-radius", type=float, default=0.08)
    parser.add_argument("--outlier-filter-min-neighbors", type=int, default=20)
    parser.add_argument("--chunk-points", type=int, default=1000000)
    return parser


def validate_args(args):
    if os.path.abspath(args.input) == os.path.abspath(args.output):
        raise RuntimeError("Input and output PCD paths must differ")
    if args.ground_min > args.height_max:
        raise RuntimeError("ground_min must not exceed height_max")
    if args.voxel_res <= 0.0:
        raise RuntimeError("voxel_res must be positive")
    if args.min_voxel_points < 1:
        raise RuntimeError("min_voxel_points must be at least one")
    if args.outlier_filter_radius <= 0.0:
        raise RuntimeError("outlier_filter_radius must be positive")
    if args.outlier_filter_min_neighbors < 1:
        raise RuntimeError("outlier_filter_min_neighbors must be at least one")
    if args.chunk_points < 1:
        raise RuntimeError("chunk_points must be positive")


def main():
    args = build_argument_parser().parse_args()
    try:
        total_started = time.perf_counter()
        validate_args(args)
        raw_points, fields, xyz_indices, _ = parse_binary_pcd(args.input)
        rotation, translation = initial_transform(
            args.robot_map_x, args.robot_map_y, args.robot_map_z,
            args.map_roll, args.map_pitch, args.map_yaw)
        print("Input: %s" % os.path.abspath(args.input))
        print("Loaded: %d points, fields=%s" % (len(raw_points), fields))
        print(
            "Preprocess transform: robot_map=(%.6f, %.6f, %.6f), rpy_deg=(%.6f, %.6f, %.6f)"
            % (args.robot_map_x, args.robot_map_y, args.robot_map_z,
               args.map_roll, args.map_pitch, args.map_yaw))

        stage_started = time.perf_counter()
        world_points = transform_and_height_filter(
            raw_points, xyz_indices, rotation, translation,
            args.ground_min, args.height_max, args.chunk_points)
        print("Height filter: %d -> %d points (%.3fs)" %
              (len(raw_points), len(world_points), time.perf_counter() - stage_started))

        if args.outlier_filter_enable and len(world_points):
            stage_started = time.perf_counter()
            before = len(world_points)
            world_points = filter_sparse_outliers(
                world_points, args.outlier_filter_radius,
                args.outlier_filter_min_neighbors)
            print("Sparse-outlier filter: %d -> %d points (%.3fs)" %
                  (before, len(world_points), time.perf_counter() - stage_started))

        stage_started = time.perf_counter()
        planner_world_points, before_density, density_count = density_filter_and_downsample(
            world_points, args.voxel_res, args.min_voxel_points)
        print("Voxel density filter: %d -> %d points" %
              (before_density, density_count))
        print("Voxel downsample: %d -> %d points (density + downsample %.3fs)" %
              (density_count, len(planner_world_points),
               time.perf_counter() - stage_started))
        if not len(planner_world_points):
            raise RuntimeError("Preprocessing produced an empty planner map")

        stored_points = world_to_map_input(
            planner_world_points, rotation, translation)
        reconstructed_world_points = np.asarray(
            (rotation @ stored_points.T).T + translation, dtype=np.float32)
        roundtrip_delta = (
            reconstructed_world_points.astype(np.float64) -
            planner_world_points.astype(np.float64))
        roundtrip_l2 = np.linalg.norm(roundtrip_delta, axis=1)
        write_xyz_binary_pcd(args.output, stored_points, args)

        # Reopen the artifact so malformed headers or partial writes fail here,
        # rather than much later during a robot launch.
        verified, verified_fields, _, _ = parse_binary_pcd(args.output)
        if verified_fields != ["x", "y", "z"] or len(verified) != len(stored_points):
            raise RuntimeError("Output verification failed")
        stored_min, stored_max = ranges(stored_points)
        world_min, world_max = ranges(planner_world_points)
        print("Output: %s" % os.path.abspath(args.output))
        print("Output bytes: %d" % os.path.getsize(args.output))
        print("Stored map-input XYZ min=%s max=%s" % (stored_min, stored_max))
        print("Expected runtime world XYZ min=%s max=%s" % (world_min, world_max))
        print(
            "Legacy/new runtime point-set error: max_abs=%.9gm max_l2=%.9gm mean_l2=%.9gm"
            % (np.max(np.abs(roundtrip_delta)), np.max(roundtrip_l2),
               np.mean(roundtrip_l2)))
        print("SHA256: %s" % sha256_file(args.output))
        print("Total preprocessing time: %.3fs; peak RSS: %.1f MiB" %
              (time.perf_counter() - total_started,
               resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0))
        print(
            "Runtime semantics: keep the same robot_map/map_rpy values and set "
            "input_is_preprocessed=true; do not zero robot_map_z for this artifact.")
    except Exception as error:
        print("ERROR: %s" % error, file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
