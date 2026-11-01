#!/usr/bin/env python3
"""Compare new cheap physical gradients to the previously attested C++ probe."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from rootcause_components import Probe, attest, context


def verify(folder):
    folder = Path(folder).resolve()
    plan = json.loads((folder / 'input.json').read_text())
    actual = [json.loads(line) for line in (folder / 'results.jsonl').open()]
    trial = Path(plan['source_proof']['case_path']).parents[1]
    parameters = json.loads((trial / 'effective_real_params.json').read_text())
    proof = attest()
    probe = Probe(proof['library'])
    rows = []
    for item in plan['checks']:
        measured = next(r for r in actual if r['kind'] == 'check' and r['label'] == item['label'])
        run = dict(**item['input'], stages=[item['stage']])
        ctx = context(run, parameters)
        old = probe.evaluate(np.asarray(item['stage']['x'], dtype=float), ctx)
        expected_cost = sum(old['costs'][k] for k in ('smooth', 'wheel', 'time'))
        expected_gradient = sum(np.asarray(old['gradients'][k]) for k in ('smooth', 'wheel', 'time'))
        got = np.asarray(measured['cheap_proxy_gradient'])
        error = abs(got - expected_gradient)
        relative = error / np.maximum(1., np.maximum(abs(got), abs(expected_gradient)))
        cost_error = abs(measured['cheap_proxy_cost'] - expected_cost) / max(1., abs(expected_cost))
        ok = cost_error < 1e-9 and float(np.max(relative)) < 1e-7
        rows.append(dict(label=item['label'], passed=bool(ok), proxy_cost=measured['cheap_proxy_cost'],
            independently_replayed_cost=expected_cost, relative_cost_error=cost_error,
            max_absolute_gradient_error=float(np.max(error)),
            max_normalized_gradient_error=float(np.max(relative)),
            compared_free_variables=int(len(ctx.free_indices)),
            full_3d_objective_used_for_scale_estimation=False))
    result = dict(complete=True, passed=all(r['passed'] for r in rows), checks=rows,
        independent_probe=proof,
        source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        results_sha256=hashlib.sha256((folder/'results.jsonl').read_bytes()).hexdigest(),
        input_sha256=hashlib.sha256((folder/'input.json').read_bytes()).hexdigest())
    (folder/'independent_proxy_verification.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(dict(passed=result['passed'], checks=rows), indent=2), flush=True)
    if not result['passed']:
        raise RuntimeError('Cheap proxy differs from the independently attested native components')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder', type=Path)
    verify(parser.parse_args().folder)
