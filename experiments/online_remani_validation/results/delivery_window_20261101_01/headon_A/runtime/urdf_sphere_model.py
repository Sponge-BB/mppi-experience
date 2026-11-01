#!/usr/bin/env python3
"""Generate isolated, URDF-derived spheres for the fixed robot structure.

This does not change REMANI, load ROS, or certify the real robot.  Meshes use
the circum-sphere of each cell in their collision-local AABB.  Cylinders use
their exact analytic enclosing sphere, not an unnecessarily larger AABB ball.
Moving manipulator links are deliberately excluded.
"""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np


FIXED_LINKS = (
    "base_link", "fr_wheel", "fl_wheel", "br_wheel", "bl_wheel",
    "arm_support", "arm_base_link", "arm_base", "velodyne_base_link",
)
PROFILES = {
    "coarse": {
        "base_link": (4, 4, 1),
        "arm_support": (2, 2, 4),
        "arm_base_link": (3, 3, 1),
    },
    "compact": {
        "base_link": (5, 5, 1),
        "arm_support": (3, 3, 5),
        "arm_base_link": (3, 3, 1),
    },
}


def load_stl(path):
    """Return STL triangles as an ``(N, 3, 3)`` float64 array."""
    data = Path(path).read_bytes()
    if len(data) >= 84:
        count = struct.unpack_from("<I", data, 80)[0]
        if len(data) == 84 + 50 * count:
            records = np.frombuffer(data, offset=84, count=count, dtype=np.dtype([
                ("normal", "<f4", (3,)),
                ("vertices", "<f4", (3, 3)),
                ("attribute", "<u2"),
            ]))
            triangles = records["vertices"].astype(float)
            if not len(triangles) or not np.isfinite(triangles).all():
                raise ValueError("Empty or non-finite STL: " + str(path))
            return triangles
    vertices = []
    for line in data.decode("ascii").splitlines():
        words = line.split()
        if words and words[0] == "vertex":
            if len(words) != 4:
                raise ValueError("Malformed ASCII STL vertex: " + str(path))
            vertices.append([float(x) for x in words[1:]])
    triangles = np.asarray(vertices, dtype=float)
    if (not len(vertices) or len(vertices) % 3
            or not np.isfinite(triangles).all()):
        raise ValueError("Invalid ASCII STL: " + str(path))
    return triangles.reshape(-1, 3, 3)


def origin_transform(origin):
    """URDF origin transform: translation followed by Rz(yaw)Ry(pitch)Rx(roll)."""
    result = np.eye(4)
    if origin is None:
        return result
    xyz = np.asarray([float(x) for x in origin.get("xyz", "0 0 0").split()])
    rpy = np.asarray([float(x) for x in origin.get("rpy", "0 0 0").split()])
    if xyz.shape != (3,) or rpy.shape != (3,):
        raise ValueError("URDF origin requires three xyz/rpy entries")
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = (
        math.cos(r), math.sin(r), math.cos(p), math.sin(p),
        math.cos(y), math.sin(y),
    )
    result[:3, :3] = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])
    result[:3, 3] = xyz
    return result


def fixed_transforms(root):
    """Map link names to base_link transforms at zero joint positions.

    The selected fixed structure contains four continuous wheel joints. Their
    collision cylinders are rotation-symmetric about their wheel axes, so zero
    wheel rotation is sufficient here. This is NOT a motion envelope for arm
    joints; no moving arm link is included by ``build_model``.
    """
    transforms = {"base_link": np.eye(4)}
    children = {}
    for joint in root.findall("joint"):
        parent, child = joint.find("parent"), joint.find("child")
        if parent is None or child is None:
            raise ValueError("Joint without parent/child")
        children.setdefault(parent.get("link"), []).append(
            (child.get("link"), origin_transform(joint.find("origin"))))
    pending = ["base_link"]
    while pending:
        parent = pending.pop()
        for child, transform in children.get(parent, []):
            if child in transforms:
                raise ValueError("URDF link has repeated parent/cycle: " + child)
            transforms[child] = transforms[parent] @ transform
            pending.append(child)
    return transforms


def _package_root(urdf_path):
    for directory in urdf_path.resolve().parents:
        package_xml = directory / "package.xml"
        if package_xml.is_file():
            return directory, ET.parse(package_xml).getroot().findtext("name")
    raise ValueError("Cannot locate URDF owning package.xml: " + str(urdf_path))


def _mesh_source(filename, package_dir, package_name, link):
    prefix = "package://" + package_name + "/"
    if filename.startswith(prefix):
        requested = package_dir / filename[len(prefix):]
    elif filename.startswith("file://"):
        requested = Path(filename[len("file://"):])
    elif not filename.startswith("package://"):
        requested = package_dir / filename
    else:
        raise ValueError("Unsupported external-package mesh: " + filename)
    fallback = None
    if not requested.is_file():
        # A known broken absolute /home/hai/... symlink in this URDF. Do not
        # repair it or silently apply this fallback to unrelated missing files.
        if (link == "arm_base"
                and filename == prefix + "meshes/arm/dae/base_link.stl"):
            actual = package_dir / "meshes/arm/base_link.stl"
            if actual.is_file():
                fallback = {
                    "reason": "URDF collision mesh path missing/broken symlink",
                    "requested_path": str(requested),
                    "requested_is_symlink": requested.is_symlink(),
                    "requested_symlink_target": (
                        os.readlink(str(requested)) if requested.is_symlink() else None),
                    "used_path": str(actual.resolve()),
                    "production_path_modified": False,
                }
                return actual.resolve(), fallback
        raise FileNotFoundError("Missing URDF collision mesh: " + str(requested))
    return requested.resolve(), fallback


def _sphere(link, collision_index, center, radius, bounds, transform,
            ground_offset, coverage_type, cell_index):
    base = (transform @ np.r_[center, 1.0])[:3]
    ground = base + np.array([0.0, 0.0, ground_offset])
    return {
        "link": link,
        "collision_index": collision_index,
        "cell_index": list(cell_index),
        "center_collision_local": np.asarray(center).tolist(),
        "center_base_link": base.tolist(),
        "center_ground": ground.tolist(),
        "radius": float(radius),
        "coverage_cell_local": {
            "min": np.asarray(bounds[0]).tolist(),
            "max": np.asarray(bounds[1]).tolist(),
        },
        "coverage_type": coverage_type,
    }


def build_model(urdf_path, profile="coarse", ground_offset=0.115):
    """Build a JSON-serializable collision-model candidate; do not write files.

    ``radius`` is geometry only: no map-resolution compensation, optimizer
    margin, or new safety inflation is added. Coarse/compact produce 47/85
    spheres for the current mm.urdf. These candidates are not enabled anywhere.
    """
    if profile not in PROFILES:
        raise ValueError("Unknown profile: " + str(profile))
    if not math.isfinite(ground_offset):
        raise ValueError("ground_offset must be finite")
    urdf_path = Path(urdf_path).resolve()
    root = ET.parse(urdf_path).getroot()
    transforms = fixed_transforms(root)
    package_dir, package_name = _package_root(urdf_path)
    links = {link.get("name"): link for link in root.findall("link")}
    spheres, components, warnings = [], [], []
    for name in FIXED_LINKS:
        if name not in links or name not in transforms:
            raise ValueError("Selected structure missing from URDF: " + name)
        collisions = links[name].findall("collision")
        if not collisions:
            raise ValueError("Selected link has no collision geometry: " + name)
        for collision_index, collision in enumerate(collisions):
            geometries = list(collision.find("geometry"))
            if len(geometries) != 1:
                raise ValueError("Expected one shape in collision geometry: " + name)
            geometry = geometries[0]
            transform = (transforms[name]
                         @ origin_transform(collision.find("origin")))
            component = {
                "link": name,
                "collision_index": collision_index,
                "geometry_kind": geometry.tag,
                "base_link_T_collision": transform.tolist(),
                "base_link_T_link": transforms[name].tolist(),
                "link_T_collision": origin_transform(collision.find("origin")).tolist(),
                "triangle_count": 0,
                "source_path": None,
                "fallback": None,
            }
            if geometry.tag == "mesh":
                filename = geometry.get("filename")
                path, fallback = _mesh_source(
                    filename, package_dir, package_name, name)
                triangles = load_stl(path)
                scale = np.array([float(x) for x in geometry.get(
                    "scale", "1 1 1").split()])
                if scale.shape != (3,) or not np.isfinite(scale).all() or np.any(scale == 0):
                    raise ValueError("Invalid URDF mesh scale: " + name)
                vertices = (triangles * scale).reshape(-1, 3)
                lower, upper = vertices.min(axis=0), vertices.max(axis=0)
                component.update({
                    "triangle_count": int(len(triangles)),
                    "source_path": str(path),
                    "source_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                    "requested_mesh_uri": filename,
                    "mesh_scale": scale.tolist(),
                    "fallback": fallback,
                    "coverage_contract": "entire collision-local AABB volume",
                })
                if fallback:
                    warnings.append(name + ": explicit mesh fallback; broken production symlink unchanged")
                divisions = PROFILES[profile].get(name, (1, 1, 1))
                component["divisions"] = list(divisions)
                widths = (upper - lower) / np.array(divisions)
                for index in np.ndindex(divisions):
                    cell_lo = lower + np.array(index) * widths
                    cell_hi = cell_lo + widths
                    center = (cell_lo + cell_hi) / 2
                    radius = np.linalg.norm(cell_hi - cell_lo) / 2
                    spheres.append(_sphere(
                        name, collision_index, center, radius,
                        (cell_lo, cell_hi), transform, ground_offset,
                        "aabb_cell_circumsphere", index))
            elif geometry.tag == "cylinder":
                radius, length = float(geometry.get("radius")), float(geometry.get("length"))
                if radius <= 0 or length <= 0 or not math.isfinite(radius + length):
                    raise ValueError("Invalid cylinder: " + name)
                lower = np.array([-radius, -radius, -length / 2])
                upper = -lower
                component.update({
                    "cylinder_radius": radius,
                    "cylinder_length": length,
                    "divisions": [1, 1, 1],
                    "coverage_contract": "analytic cylinder volume, not all AABB corners",
                })
                spheres.append(_sphere(
                    name, collision_index, np.zeros(3),
                    math.hypot(radius, length / 2), (lower, upper),
                    transform, ground_offset, "analytic_cylinder", (0, 0, 0)))
            else:
                raise ValueError("Unsupported selected collision shape: " + geometry.tag)
            component["local_bounds"] = {"min": lower.tolist(), "max": upper.tolist()}
            components.append(component)
    for index, sphere in enumerate(spheres):
        sphere["id"] = index
    urdf_arm_height = float(transforms["arm_base"][2, 3] + ground_offset)
    warnings.extend([
        "Fixed structure only; moving arm links are excluded, not replaced or certified.",
        "URDF names a VLP16 shell while real launch defaults to Livox; confirm actual sensor housing.",
        "Sphere radius is geometric coverage only; existing safety margins are not modified.",
        "Spheres can extend below ground or beyond mesh bounds; this candidate is not deployed.",
    ])
    return {
        "schema_version": 1,
        "profile": profile,
        "urdf_path": str(urdf_path),
        "urdf_sha256": hashlib.sha256(urdf_path.read_bytes()).hexdigest(),
        "sphere_count": len(spheres),
        "included_links": list(FIXED_LINKS),
        "components": components,
        "spheres": spheres,
        "frame_contract": {
            "sphere_center_base_link": "URDF base_link coordinates, metres",
            "sphere_center_ground": "same x/y; z adds ground_offset",
            "ground_offset": ground_offset,
            "urdf_arm_base_height_ground": urdf_arm_height,
            "current_planner_arm_base_height_ground_reference": 0.68,
            "urdf_minus_planner_arm_height_reference": urdf_arm_height - 0.68,
            "reference_note": "0.68 m is current Piper config reference, not a parsed universal URDF property",
        },
        "safety_margin_change": 0.0,
        "enabled_in_planner": False,
        "warnings": warnings,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--profile", choices=tuple(PROFILES), default="coarse")
    parser.add_argument("--ground-offset", type=float, default=0.115)
    args = parser.parse_args()
    model = build_model(args.urdf, args.profile, args.ground_offset)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(model, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({"output": str(args.output), "profile": model["profile"],
                      "sphere_count": model["sphere_count"]}, ensure_ascii=False))


if __name__ == "__main__":
    main()
