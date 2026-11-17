#!/usr/bin/python3
"""Artifact-only Q_START override; retain the original simulator entry guards."""
import json
import os
from pathlib import Path
import sys
from urllib.parse import urlparse

import numpy as np

ROOT = Path(__file__).resolve().parents[4]
SCRIPT = ROOT/'src/mm_remani/scripts/standby_simulator.py'
if len(sys.argv) < 2 or Path(sys.argv[1]).resolve() != SCRIPT:
    raise RuntimeError('launch-prefix only supports the current standby_simulator.py')
sys.argv = sys.argv[1:]
sys.path.insert(0, str(SCRIPT.parent))
import standby_simulator as simulator

record = ROOT/'experiments/online_remani_validation/results/hardware_startup_check_20261107/runtime_feedback.json'
values = json.loads(record.read_text())['topics']['/joint_states_single']
if values['names'] != ['joint%d' % i for i in range(1, 7)]:
    raise ValueError('recorded joint order must be joint1 through joint6')
simulator.Q_START = np.asarray(values['position_rad'], dtype=float)
if simulator.Q_START.shape != (6,) or not np.isfinite(simulator.Q_START).all():
    raise ValueError('invalid recorded initial joint posture')

# These are the production module's __main__ checks, before any simulator node.
master = urlparse(os.environ.get('ROS_MASTER_URI', ''))
if master.hostname not in ('localhost', '127.0.0.1') or master.port != 11466:
    raise RuntimeError('simulation requires isolated localhost ROS master port 11466')
simulator.rospy.init_node('standby_simulator')
if not simulator.rospy.get_param('/use_sim_time', False):
    raise RuntimeError('/use_sim_time must be true')
simulator.StandbySimulator().run()
