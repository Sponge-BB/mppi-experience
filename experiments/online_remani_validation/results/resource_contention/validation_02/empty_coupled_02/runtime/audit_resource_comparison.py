#!/usr/bin/env python3
"""Check CPU data coverage, isolation and immutable binaries, not goal success."""
import argparse
import collections
import hashlib
import json
import math
from pathlib import Path


def read_lines(path):return [json.loads(s) for s in path.read_text().splitlines() if s.strip()]
def require(value,message):
    if not value:raise RuntimeError(message)


def audit(out):
    plan=json.loads((out/'experiment_plan.json').read_text())
    expected=len(plan['scenes'])*len(plan['groups'])*plan['repeats']
    paths=sorted(p for p in out.iterdir() if p.is_dir() and p.name!='paired_static_diagnostic' and (p/'run_status.json').exists())
    require(len(paths)==expected,'incomplete main matrix')
    native_hashes=set();mppi_hashes=set();coverage=[];outcomes=collections.defaultdict(collections.Counter)
    solver_count=0;cpu_samples=0
    for path in paths:
        manifest=json.loads((path/'manifest.json').read_text())
        status=json.loads((path/'run_status.json').read_text());result=json.loads((path/'result.json').read_text())
        require(not status['required_process_failed'],'native process failed: '+path.name)
        native_hashes.add(json.dumps(manifest['native_binaries_sha256'],sort_keys=True))
        mppi_hashes.add(manifest['mppi_library_sha256'])
        samples=read_lines(path/'cpu_samples.jsonl');traces=read_lines(path/'solver_trace.jsonl')
        names=collections.Counter(p['name'] for s in samples for p in s['processes'])
        require(names['remani_planner_node']>len(samples)*.6,'missing native CPU: '+path.name)
        group=manifest['resource_group']
        mppi='background_mppi' if group=='background' else 'base_mppi'
        if group!='remani':require(names[mppi]>len(samples)*.6,'missing MPPI CPU: '+path.name)
        pub=json.loads((path/'publisher_audit.json').read_text())
        if group=='background':
            require(not pub['background_nonisolated_publishers'],'background output leakage')
            require(pub['publishers'].get('/sim/cmd_vel')==['/real_base_tracker'],'background altered command owner')
        elif group=='coupled':require(pub['publishers'].get('/sim/cmd_vel')==['/base_mppi'],'wrong coupled command owner')
        begins={(r['stage'],r['id']) for r in traces if r['event']=='begin'}
        ends={(r['stage'],r['id']) for r in traces if r['event']=='end'}
        require(begins==ends,'incomplete solver accounting: '+path.name)
        for r in traces:
            if r['event']!='end':continue
            require(all(math.isfinite(r[k]) and r[k]>=0 for k in ['wall_ms','thread_cpu_ms','process_cpu_ms','runqueue_wait_ms']), 'invalid accounting value')
            require(r['thread_cpu_ms']<r['wall_ms']+2.,'thread CPU exceeds wall time')
        coverage.append({'trial':path.name,'cpu_samples':len(samples),'native_samples':names['remani_planner_node'],
                         'mppi_samples':names[mppi] if group!='remani' else 0})
        outcomes[group][result['reason']]+=1;solver_count+=len(ends);cpu_samples+=len(samples)
    require(len(native_hashes)==1 and len(mppi_hashes)==1,'binary mismatch between groups')
    pair=out/'paired_static_diagnostic'
    paired=[r for r in read_lines(pair/'solver_trace.jsonl') if r['event']=='end' and r['stage']=='optimizer' and r['label'].startswith('paired_')]
    require(len(paired)==8,'incomplete paired optimizer diagnostic')
    require(len({r['input_hash'] for r in paired})==1,'paired optimizer input differs')
    for budget in [.6,1.5]:
        for label in ['paired_mppi_paused','paired_mppi_active']:
            require(sum(r['label']==label and abs(r['budget_s']-budget)<1e-8 for r in paired)==2,'unbalanced paired diagnostic')
    manifest=json.loads((Path(plan['build'])/'build_manifest.json').read_text())
    root=Path(__file__).resolve().parents[2]
    changed=[p for p,h in manifest['production_sources_sha256'].items() if hashlib.sha256((root/p).read_bytes()).hexdigest()!=h]
    require(not changed,'production source changed: '+str(changed))
    calibration=json.loads((out.parent/'scheduler_calibration.json').read_text())
    require(calibration['queue_counter_tracks_contention'],'runqueue counter calibration failed')
    result={'passed':True,'main_trials':len(paths),'cpu_samples':cpu_samples,'solver_scopes':solver_count,
            'paired_calls':len(paired),'paired_input_hash':paired[0]['input_hash'],
            'outcomes':outcomes,'coverage':coverage,'production_sources_changed':changed,
            'checks':['required CPU PIDs and sample coverage','same binary hashes','background output isolation',
                      'single expected command owner','complete begin/end accounting','finite CPU and wall time',
                      'identical paired optimizer request','balanced paused/active budgets','scheduler counter calibration',
                      'production source hashes preserved'],
            'interpretation':'data and experiment-contract audit; collisions and task failures are retained'}
    (out/'resource_audit.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='coverage'},ensure_ascii=False,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('out',type=Path);audit(p.parse_args().out.resolve())
