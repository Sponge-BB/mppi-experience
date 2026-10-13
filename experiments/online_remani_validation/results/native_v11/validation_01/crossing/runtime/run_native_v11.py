#!/usr/bin/env python3
"""Run V11 in the private numerical ROS simulator, never a hardware launch."""
import argparse
from pathlib import Path
from run import run

HERE=Path(__file__).resolve().parent

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',type=Path,required=True,help='new result directory')
    parser.add_argument('--scene',choices=['empty','static','crossing'],default='static')
    parser.add_argument('--duration',type=float,default=52.)
    parser.add_argument('--reference-delay',type=float,default=0.)
    parser.add_argument('--build',type=Path,default=HERE/'builds/native_v11_03')
    parser.set_defaults(mode='mppi',perception='classified',execution='local',projection_progress=True,
        planner_variant='current',scenario_start='motion',debug_native=False)
    run(parser.parse_args())
