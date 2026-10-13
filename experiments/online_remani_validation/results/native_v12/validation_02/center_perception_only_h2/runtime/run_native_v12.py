#!/usr/bin/env python3
"""V12 explicit opt-in experiment runner; preserves V11 defaults and records."""
import argparse
from pathlib import Path
from run import run
from dynamic_scenarios import SCENES
from render_native_search import render
HERE=Path(__file__).resolve().parent

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,required=True)
    p.add_argument('--build',type=Path,default=HERE/'builds/native_v12_05')
    p.add_argument('--scene',choices=SCENES,default='dynamic_head_on_center')
    p.add_argument('--duration',type=float,default=60.)
    p.add_argument('--prediction-horizon',type=float,choices=[2.,3.,4.],default=None)
    p.add_argument('--reference-delay',type=float,default=0.)
    p.add_argument('--no-record-search',dest='record_search',action='store_false')
    p.add_argument('--no-render',action='store_true')
    p.set_defaults(mode='mppi',perception='classified',execution='local',projection_progress=True,
                   planner_variant='current',scenario_start='motion',debug_native=False)
    a=p.parse_args();run(a)
    if not a.no_render:render(a.out.resolve(),a.out.resolve()/'visualization')
