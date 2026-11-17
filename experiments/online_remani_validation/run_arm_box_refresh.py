#!/usr/bin/env python3
"""Paired arm-box trials using immutable legacy or refresh-only child planner."""
import argparse
import json
from pathlib import Path

import run_arm_box_navigation as runner

HERE=Path(__file__).resolve().parent
CASE=HERE/'results/arm_box_refresh_20261115-160051'


def prepare(kind, seed, duration=45.):
    if kind=='fixed':
        from build_arm_box_refresh import check,OUT
        if check()['prepared_only']:raise RuntimeError('Refresh child has not been compiled')
    runner.CASE=CASE/('%s_seed%d'%(kind,seed))
    out=runner.prepare('mppi',duration,seed)
    p=out/'manifest.json';m=json.loads(p.read_text())
    m.update(refresh_variant=kind,paired_seed=seed)
    if kind=='fixed':
        link=out/'ros_pkgs/online_remani_experiment/remani_planner_node'
        link.unlink();link.symlink_to(OUT/'build/remani_planner_node')
        m['actual_planner_sha256']=runner.sha(link)
        m['native_binaries_sha256']['remani_planner_node']=runner.sha(link)
        m['refresh_child_manifest_sha256']=runner.sha(OUT/'build_manifest.json')
        m['refresh_child_build']=str(OUT)
    p.write_text(json.dumps(m,ensure_ascii=False,indent=2)+'\n')
    return out


def run(kind,seed,duration=45.):
    runner.CASE=CASE/('%s_seed%d'%(kind,seed))
    if not (runner.CASE/'mppi').exists():prepare(kind,seed,duration)
    return runner.run('mppi',duration,seed)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('kind',choices=['legacy','fixed']);p.add_argument('--seed',type=int,default=11)
    p.add_argument('--duration',type=float,default=45.);p.add_argument('--prepare-only',action='store_true')
    p.add_argument('--case',type=Path,default=CASE)
    a=p.parse_args();CASE=a.case.resolve()
    (prepare if a.prepare_only else run)(a.kind,a.seed,a.duration)
