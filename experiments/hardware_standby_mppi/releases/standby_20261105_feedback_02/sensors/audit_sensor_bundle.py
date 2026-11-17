#!/usr/bin/python3
"""Read-only audit: isolated sensor files, dependency paths and old log hashes."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET


def main():
    sensors = Path(__file__).resolve().parent
    allowed = {'raw_sensors.launch': {'feedback02_livox_driver', 'feedback02_livox_to_pcl'},
        'sensor_odometry.launch': {'feedback02_fastlio_mapping', 'feedback02_fastlio_to_odom.py', 'static_transform_publisher'}}
    for filename, types in allowed.items():
        tree = ET.parse(sensors/filename)
        assert not list(tree.iter('include')), 'external launch included'
        assert all(n.get('type') in types for n in tree.iter('node'))
        assert all(g.get('ns') == '/standby_trial_20261105_feedback_02/sensors' for g in tree.iter('group'))
    baseline = json.loads((sensors/'old_fastlio_log_baseline.json').read_text())
    differences = []
    for name, before in baseline.items():
        p = Path(name)
        if not p.is_file() or p.stat().st_size != before['size'] or hashlib.sha256(p.read_bytes()).hexdigest() != before['sha256']:
            differences.append(name)
    assert not differences, 'original FASTLIO logs changed: '+str(differences)
    dependencies = {}
    for name in ('feedback02_livox_driver', 'feedback02_livox_to_pcl', 'feedback02_fastlio_mapping'):
        p = sensors/'bin'/name
        if not p.is_file():
            dependencies[name] = 'not_built'
            continue
        text = subprocess.run(['ldd', str(p)], text=True, capture_output=True, check=True).stdout
        assert 'not found' not in text and '/devel/' not in text, text
        dependencies[name] = 'system_and_ros_libraries_only'
    manifest_path = sensors/'build_manifest.json'
    status = json.loads(manifest_path.read_text())['status'] if manifest_path.exists() else 'not_started'
    print(json.dumps(dict(sensor_nodes_only=True, original_log_hashes_unchanged=len(baseline),
        dependencies=dependencies, isolated_fastlio_build=status, hardware_started_by_audit=False), indent=2))


if __name__ == '__main__':
    main()
