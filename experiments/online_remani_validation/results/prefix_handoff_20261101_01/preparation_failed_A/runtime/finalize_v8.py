#!/usr/bin/env python3
"""Check the V8 optimization equivalence and format plots from frozen records."""
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE / 'results/mppi_v8'
OUT = ROOT / 'validation_03'


def main():
    checks = []
    for case in ('01_single', '04_four_dense'):
        for seed in (1, 2, 3):
            old = json.loads((ROOT / 'development_08' / f'{case}_seed{seed}.json').read_text())
            new = json.loads((OUT / case / 'data' / f'v8_seed{seed}.json').read_text())
            assert len(old['trace']) == len(new['trace'])
            for a, b in zip(old['trace'], new['trace']):
                for field in ('state', 'next_state', 'command'):
                    assert a[field] == b[field], (case, seed, field)
            checks.append(dict(case=case, seed=seed, cycles=len(new['trace']), exact_match=True))
    (ROOT / 'optimization_equivalence_08_09.json').write_text(json.dumps({
        'scope': 'Exact state/next_state/command equality on six existing paired trials; not a general equivalence proof.',
        'checks': checks}, indent=2) + '\n')

    # Only presentation is regenerated; raw records, metrics and manifests stay frozen.
    for case in sorted(OUT.glob('*/data')):
        fig, axes = plt.subplots(1, 3, figsize=(11, 3.5), constrained_layout=True)
        for version, color in (('v7', '#888888'), ('v8', '#237fa4')):
            record = json.loads((case / f'{version}_seed1.json').read_text())
            rows = record['trace']
            t = np.array([r['t'] for r in rows])
            s = np.array([r['state'] for r in rows])
            axes[0].plot(s[:, 0], s[:, 1], color=color, label=version.upper())
            axes[1].plot(t, s[:, 3], color=color)
            # This is the same recorded sampled physical body clearance, not inflated distance.
            axes[2].plot(t, [r['gap'] for r in rows], color=color)
        axes[0].set(xlabel='x / m', ylabel='y / m', title='Executed center path')
        axes[0].set_aspect('equal', adjustable='datalim')
        axes[0].legend(loc='best', fontsize=8)
        axes[1].set(xlabel='time / s', ylabel='m / s', title='Actual speed')
        axes[2].set(xlabel='time / s', ylabel='m', title='Body clearance')
        axes[2].axhline(.1, color='#aaaaaa', ls='--', lw=1)
        for ax in axes:
            ax.grid(alpha=.2)
        fig.suptitle('V7 / V8 | seed 1 | fixed-reference SIL', fontsize=12)
        fig.savefig(case.parent / 'plots/comparison.png', dpi=140)
        plt.close(fig)
    print('PASS: six exact paired traces; seven comparison figures formatted.')


if __name__ == '__main__':
    main()
