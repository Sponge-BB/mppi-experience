#!/usr/bin/env python3
"""Predeclared native REMANI / V12 / shared-contract closed-loop comparisons."""
import argparse
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from run import run
from native_reference import reference_polynomial,sampled_reference,fingerprint

HERE=Path(__file__).resolve().parent
VARIANTS=('remani','v12','remani_shared','v12_shared')
SCENES=('empty','static','dynamic_single','dynamic_head_on_center','dynamic_four')


def publication_references(out):
    """Baseline has no MPPI consumer: derive a labelled publication replay."""
    if json.loads((out/'manifest.json').read_text())['mode']!='remani':return
    raw=json.loads((out/'trajectories.json').read_text());refs=[];times=[];messages=[];records=[];events=[]
    revision=0;epoch=0;start=0.
    for item in raw:
        if item['action'] in (3,4):continue
        revision+=1
        if item['action'] in (2,5):
            refs=[];times=[];messages=[]
            events.append(dict(t=item['t'],revision=revision,epoch=epoch,event='cancel'));continue
        if item['action']!=1:continue
        if item['id']==1:refs=[];times=[];messages=[];epoch+=1;start=item['stamp']
        pieces=[SimpleNamespace(**p) for p in item['polynomials']]
        rows=reference_polynomial(SimpleNamespace(trajectory=pieces,singul=item['singul']))
        offset=times[-1][-1] if times else 0.
        refs.append(rows[:,1:5]);times.append(rows[:,0]+offset);messages.append(item['message_sha256'])
        reference=sampled_reference(refs,0.)
        records.append(dict(t=item['t'],revision=revision,epoch=epoch,stamp=start,
            source='/planning/trajectory',transport='recorded native publication, NOT a controller consumption acknowledgement',
            message_hashes=messages.copy(),sha256=fingerprint(reference),reference=reference.tolist(),
            times=np.concatenate(times).tolist()))
        events.append(dict(t=item['t'],revision=revision,epoch=epoch,event='replace' if item['id']==1 else 'append'))
    for name,values in (('consumed_references.jsonl',records),('reference_events.jsonl',events)):
        with (out/name).open('x') as stream:
            for item in values:stream.write(json.dumps(item)+'\n')
    (out/'reference_recording_scope.json').write_text(json.dumps(dict(
        source='native publisher messages captured by numerical simulator',
        baseline_controller='actual mm_controller_node and RealBaseTracker ROS nodes',
        no_controller_ack_inferred=True),indent=2)+'\n')


def specs(old,new):
    values=[]
    for scene in SCENES:
        for variant in VARIANTS:
            values.append(dict(key=scene+'/'+variant,scene=scene,variant=variant,
                build=str(new if variant.endswith('_shared') else old),repeat=1))
    # Repeat all four static variants; retain all failures, no paired-RNG claim.
    for variant in VARIANTS:
        values.append(dict(key='static_repeat/'+variant,scene='static',variant=variant,
            build=str(new if variant.endswith('_shared') else old),repeat=2))
    return values


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,required=True)
    p.add_argument('--old-build',type=Path,default=HERE/'builds/native_v12_07')
    p.add_argument('--shared-build',type=Path,default=HERE/'builds/v12_shared_02')
    p.add_argument('--duration',type=float,default=52.);p.add_argument('--render-only',action='store_true')
    a=p.parse_args();out=a.out.resolve()
    if not a.render_only:
        out.mkdir(parents=True,exist_ok=False);plan=specs(a.old_build.resolve(),a.shared_build.resolve())
        manifest=dict(specs=plan,duration=a.duration,scenes=list(SCENES),variants=list(VARIANTS),
            timestamp='2026-10-13',reference='actual native REMANI PolynomialTraj; no fixed/synthetic reference',
            comparison='four arms separate the local executor from native collision-model changes; 2 static repetitions',
            seed='MPPI seed1; native RRT and scheduling not seeded or paired; NOT a statistical success-rate study',
            controls='same declared actors, measured-motion time origin, body disks, plant lag, wheel/speed limits, fixed arm and V12 detector',
            contract='hard margin0.10; common cubic comfort zone0.12, weight50000; native optimization uses a frozen present frame, MPPI uses time-indexed CV',
            caveats=['native-only command lease0.30s, local executor0.075s (inherited execution architectures)',
                'shared projected model changes native environmental geometry; original native3D controls retained',
                'producer reference replay for native-only; actual consumed transactions for MPPI',
                'timing includes search recording and collision-input recording; not a hard-realtime benchmark'],
            binaries={str(b.resolve()):json.loads((b/'build_manifest.json').read_text())['native_binaries_sha256'] for b in (a.old_build,a.shared_build)})
        (out/'suite_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
        for spec in plan:
            destination=out/spec['key'];native=spec['variant'].startswith('remani')
            args=SimpleNamespace(out=destination,build=Path(spec['build']),scene=spec['scene'],duration=a.duration,
                mode='remani' if native else 'mppi',perception='classified',execution='native' if native else 'local',
                projection_progress=not native,planner_variant='current',scenario_start='motion',debug_native=False,
                reference_delay=0.,record_search=not native,prediction_horizon=3.)
            print('START',spec['key'],flush=True)
            try:
                run(args);publication_references(destination)
            except Exception as exc:
                destination.mkdir(parents=True,exist_ok=True)
                (destination/'suite_error.json').write_text(json.dumps(dict(error=str(exc)))+'\n')
                print('FAILED',spec['key'],str(exc),flush=True)
            completed=[dict(key=s['key'],result=json.loads((out/s['key']/'result.json').read_text()).get('reason'))
                for s in plan if (out/s['key']/'result.json').exists()]
            (out/'progress.json').write_text(json.dumps(completed,indent=2)+'\n')
    from report_shared_collision import report
    report(out,render=True)


if __name__=='__main__':main()
