#!/usr/bin/env python3
"""Run V11 in the private numerical ROS simulator, never a hardware launch."""
import argparse
from pathlib import Path
from run import run
from dynamic_scenarios import SCENES

HERE=Path(__file__).resolve().parent

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',type=Path,required=True,help='new result directory')
    parser.add_argument('--scene',choices=SCENES,default='static')
    parser.add_argument('--duration',type=float,default=52.)
    parser.add_argument('--reference-delay',type=float,default=0.)
    parser.add_argument('--build',type=Path,default=HERE/'builds/native_v11_08')
    parser.add_argument('--no-record-search',dest='record_search',action='store_false',
                        help='disable optional 5 Hz search export for timing-only trials')
    parser.set_defaults(mode='mppi',perception='classified',execution='local',projection_progress=True,
        planner_variant='current',scenario_start='motion',debug_native=False)
    args=parser.parse_args()
    run(args)
    from render_native_search import render
    render(args.out.resolve(),args.out.resolve()/'visualization')
