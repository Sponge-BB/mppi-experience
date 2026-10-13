#!/usr/bin/env python3
"""Small shim that execs the canonical moving_obstacles_node.py
located in the package root scripts directory, so roslaunch will
still find a script at this path but the real implementation runs
from the canonical location. This avoids ambiguous duplicate files.
"""
import os, sys

# canonical script path (package root scripts)
script = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), 'scripts', 'moving_obstacles_node.py')
# exec with same interpreter
os.execv(sys.executable, [sys.executable, script] + sys.argv[1:])
