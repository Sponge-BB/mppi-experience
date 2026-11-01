#!/usr/bin/env python3
"""Snapshot and build current REMANI in an experiment-only directory."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
from kernel import generate
from ablation import generate as generate_ablation

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]


def build(out):
    out=Path(out).resolve();out.mkdir(parents=True,exist_ok=False)
    snapshot=out/'source';snapshot.mkdir()
    originals={}
    for package in ('mm_remani_planner','mm_controller','mm_remani'):
        src=ROOT/'src'/package
        for f in src.rglob('*'):
            if not f.is_file() or f.suffix not in ('.cpp','.h','.hpp','.py','.yaml','.xml','.launch'):continue
            dest=snapshot/f.relative_to(ROOT/'src');dest.parent.mkdir(parents=True,exist_ok=True)
            shutil.copy2(f,dest);originals[str(f.relative_to(ROOT))]=hashlib.sha256(f.read_bytes()).hexdigest()
    # Isolated fixed-arm optimizer: all sampled joint coordinates are fixed by
    # equal bounds, and their optimization coordinates get exactly zero gradient.
    # This keeps the native search, cost, L-BFGS, collision check and FSM intact.
    opt=snapshot/'mm_remani_planner/traj_opt/src/poly_traj_optimizer.cpp'
    source=opt.read_text();needle='    double costall = total_smoo_cost + total_obs_feas_costs.sum() + total_time_cost;'
    if source.count(needle)!=1:raise RuntimeError('optimizer insertion point changed')
    freeze='''    // Experiment-only hard elimination of fixed arm coordinates.
    // L-BFGS starts at q_fixed, and all direction/history entries stay zero.
    for(auto &gp : gradP_container)
      gp.bottomRows(opt->manipulator_dof_).setZero();
    for(auto &gg : gradGear_container)
      gg.bottomRows(opt->manipulator_dof_).setZero();
'''
    opt.write_text(source.replace(needle,freeze+needle))
    manifest={'production_sources_sha256':originals,
        'experiment_changes':['equal min/max arm joint bounds in launch','zero only arm waypoint and gear-position gradients in copied optimizer'],
        'production_modified':False,'source_root':str(snapshot)}
    manifest['mppi_kernel']=generate(out)
    generate_ablation(out)
    (out/'build_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    subprocess.run(['cmake','-S',str(HERE),'-B',str(out/'build'),'-DCMAKE_BUILD_TYPE=Release','-DPYTHON_EXECUTABLE=/usr/bin/python3','-DSNAPSHOT='+str(snapshot)],check=True)
    subprocess.run(['cmake','--build',str(out/'build'),'-j2'],check=True)
    for path,digest in originals.items():
        if hashlib.sha256((ROOT/path).read_bytes()).hexdigest()!=digest:raise RuntimeError('production changed: '+path)
    print('ISOLATED BUILD:',out,flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,required=True)
    build(p.parse_args().out)
