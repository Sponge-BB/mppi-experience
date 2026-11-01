#!/usr/bin/env python3
"""One fresh full REMANI+MPPI SIL run; never connects to hardware ROS."""
import argparse
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE/'results/state_handoff_ab_20261031_01/tools'))
import run_state_ab as previous
import build_static_agility as builder
from run_native_collision_3d import trial_kwargs
common=previous.common
OUT=HERE/'results/static_agility_20261031_01'


def patch(trial,settings,scene,enabled):
    previous.patch(trial,settings,scene,True)
    tree=ET.parse(trial/'experiment.launch')
    node=next(n for n in tree.getroot().findall('node') if n.get('name')=='base_mppi')
    common.set_env(node,'MPPI_STATIC_EARLY_DETOUR',int(enabled))
    tree.write(trial/'experiment.launch',encoding='unicode')
    scope=json.loads((builder.OUT/'build_manifest.json').read_text())['static_agility_scope']
    settings.update(static_early_detour=enabled,static_agility_scope=scope)
    path=trial/'coupling_parameters.json';previous.save(path,settings)
    mp=trial/'manifest.json';m=json.loads(mp.read_text())
    m['coupling_experiment']=settings;m['parameters_sha256']=common.lifecycle.sha(path)
    m['static_agility']=dict(enabled=enabled,independent_receiver=True,
        change_scope=scope,
        geometry_wheel_limits_budget_lease_unchanged=True,reference_handoff_unchanged=True,
        production_modified=False,scope='fixed-arm full REMANI numerical ROS SIL')
    previous.save(mp,m)


def main():
    p=argparse.ArgumentParser();p.add_argument('--scene',choices=['static','empty','midcross','headon'],default='static')
    p.add_argument('--enabled',type=int,choices=[0,1],required=True)
    p.add_argument('--tag',required=True)
    p.add_argument('--build',type=Path,default=HERE/'builds/static_agility_20261031_03');a=p.parse_args()
    builder.OUT=a.build.resolve()
    if not a.tag.replace('_','').isalnum():raise ValueError('Invalid trial tag')
    OUT.mkdir(exist_ok=True)
    common.register_scenes();m=builder.check()
    with common.clean_environment():
        valid=common.lifecycle.run_trial(OUT/a.tag,builder.OUT,'sphere_timing_'+a.scene,
            'engineering',11,55.,0.,0.,False,common.lifecycle.sha(builder.OUT/'build_manifest.json'),
            reserve_bytes=common.RESERVE+common.TRIAL_ALLOWANCE,
            prepare_hook=lambda t,s:patch(t,s,a.scene,bool(a.enabled)),**trial_kwargs(m))
    print('STATIC_AGILITY_TRIAL_COMPLETE',a.tag,valid,flush=True)
    if not valid:raise RuntimeError('Interface-invalid trial retained')


if __name__=='__main__':main()
