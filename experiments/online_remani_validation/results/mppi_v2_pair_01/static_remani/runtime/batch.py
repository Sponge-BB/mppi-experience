#!/usr/bin/env python3
"""Sequential trials: no concurrent planner benchmarks on this host."""
import argparse
import subprocess
import sys
from pathlib import Path

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--build',type=Path,required=True)
    p.add_argument('--scenes',default='empty,static,crossing');p.add_argument('--modes',default='remani,mppi,mppi_cbf');p.add_argument('--duration',default='45');p.add_argument('--projection-progress',action='store_true');p.add_argument('--planner-variant',choices=['current','no_wait'],default='current');p.add_argument('--scenario-start',choices=['goal','motion'],default='goal');a=p.parse_args()
    a.out.mkdir(parents=True,exist_ok=False)
    for scene in a.scenes.split(','):
        for mode in a.modes.split(','):
            print('RUN',scene,mode,flush=True)
            subprocess.run([sys.executable,str(Path(__file__).with_name('run.py')),'--out',str(a.out/(scene+'_'+mode)),'--build',str(a.build),'--scene',scene,'--mode',mode,'--duration',a.duration,'--planner-variant',a.planner_variant,'--scenario-start',a.scenario_start]+(['--projection-progress'] if a.projection_progress else []),check=True)
