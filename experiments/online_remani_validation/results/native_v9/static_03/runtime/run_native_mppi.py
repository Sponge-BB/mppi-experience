#!/usr/bin/env python3
"""V8 MPPI + actual online REMANI reference (isolated ROS SIL only)."""
import argparse
from pathlib import Path
from run import run

HERE=Path(__file__).resolve().parent

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out', type=Path, required=True, help='new result directory')
    p.add_argument('--scene', choices=['empty','static','crossing'], default='empty')
    p.add_argument('--duration', type=float, default=45)
    p.add_argument('--mode', choices=['remani','mppi'], default='mppi')
    p.add_argument('--build', type=Path, default=HERE/'builds/mppi_v8_09')
    p.add_argument('--perception',choices=['auto','merged','classified'],default='auto')
    p.set_defaults(projection_progress=True, planner_variant='current', scenario_start='motion', debug_native=False)
    run(p.parse_args())
