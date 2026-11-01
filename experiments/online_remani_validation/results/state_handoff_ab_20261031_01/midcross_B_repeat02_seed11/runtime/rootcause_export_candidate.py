#!/usr/bin/env python3
"""Export the nearby lower-cost candidate for independent native acceptance."""
import json
from pathlib import Path
import numpy as np

from audit_delivery_iterates import loads_preserving_cpp_zero
from rootcause_components import HERE, TRIAL, Probe, attest, context, hard_peak, sha


def main():
    source = HERE/'results/rootcause_components_20261030_02/summary.json'
    report = json.loads(source.read_text())
    row = next(s for s in report['stages'] if s['run'] == 'endpoint150_060' and s['stage'] == 'accepted')
    raw = TRIAL/'candidate_audit/endpoint_time_replay.jsonl'
    if sha(raw) != report['raw_sha256']:
        raise ValueError('Frozen capture changed')
    run = next(r for r in (loads_preserving_cpp_zero(l) for l in raw.open()) if r.get('label') == 'endpoint150_060')
    x = np.asarray(run['stages'][-1]['x'], dtype=float)
    g = np.asarray(row['component_gradients']['known_full_local'], dtype=float)
    step = 1e-5
    nearby = x-step*g/np.linalg.norm(g)
    ctx = context(run, json.loads((TRIAL/'effective_real_params.json').read_text()))
    proof = attest()
    value = Probe(proof['library']).evaluate(nearby, ctx)
    output = dict(schema='rootcause_nearby_candidate_v1', provenance_summary=str(source),
        provenance_summary_sha256=sha(source), raw_sha256=sha(raw), source_sha256=sha(__file__),
        treatment='one unit-normalized analytic descent step, not a new optimizer result',
        direction='smooth + base_feasibility + (1+self_cost/duration)*time', step=step,
        x_original=x.tolist(), x=nearby.tolist(), max_x_change=float(max(abs(nearby-x))),
        native_component_costs=value['costs'], fixed_arm_self_per_second=row['fixed_arm_self_per_second'],
        native_motion_peak=hard_peak(value, ctx),
        parts=[dict(gear=int(gear), times=times.tolist(), b_rows_ascending_power=coeff.tolist()) for gear,times,coeff in zip(ctx.gears,value['times_parts'],value['b_parts'])],
        full_native_collision_gate_replayed=False, delivered_or_published=False)
    target = source.parent/'nearby_candidate.json'
    if target.exists(): raise ValueError('Existing export would be overwritten')
    target.write_text(json.dumps(output, indent=2)+'\n')
    print(target)
    print(output['native_motion_peak'])


if __name__ == '__main__':
    main()
