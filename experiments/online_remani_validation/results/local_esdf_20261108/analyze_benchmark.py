#!/usr/bin/env python3
"""Summarize only the offline EDT timings; no ROS or scientific dependencies."""
import json
from pathlib import Path
from statistics import median
import sys


def main():
    source = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).with_name("benchmark.json")
    data = json.loads(source.read_text())
    nearby = [frame for frame in data["frames"] if frame["name"].startswith("nearby_")]
    summary = {
        "scope": data["scope"],
        "timing_exclusions": data["timing_exclusions"],
        "all_equal": data["all_equal"],
        "compared_voxels_per_frame": 3600000,
        "frames": len(data["frames"]),
        "nearby_frames": len(nearby),
        "max_abs_error_m": max([data["initial_max_abs_error_m"]] + [f["max_abs_error_m"] for f in data["frames"]]),
        "nearby": {},
        "widely_separated_changes": next(f for f in data["frames"] if f["name"] == "widely_separated_changes"),
        "unchanged_occupancy": next(f for f in data["frames"] if f["name"] == "unchanged_occupancy"),
    }
    for mode in ("full", "local"):
        summary["nearby"][mode] = {
            key: {"median": median(f[mode][key] for f in nearby),
                  "min": min(f[mode][key] for f in nearby),
                  "max": max(f[mode][key] for f in nearby)}
            for key in ("pipeline_wall_ms", "prepare_ms", "snapshot_copy_ms", "compute_ms", "commit_ms", "compute_voxels", "write_voxels")
        }
    summary["nearby"]["median_paired_speedup"] = median(f["full"]["pipeline_wall_ms"] / f["local"]["pipeline_wall_ms"] for f in nearby)
    output = source.with_name(source.stem + "_summary.json")
    output.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    if not data["all_equal"]:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
