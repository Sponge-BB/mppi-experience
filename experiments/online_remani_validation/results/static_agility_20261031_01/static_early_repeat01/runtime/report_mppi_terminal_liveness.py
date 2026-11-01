#!/usr/bin/env python3
"""Two key visuals and an auditable summary for terminal MPPI liveness.

Read-only with respect to measurement inputs. Run only after every measured
trial has ended. The GIF uses a shared 20-second SIMULATION timeline, not wall
time; completed trials hold their final measured pose without extending their
search trajectories. Neither figure is a ROS/sensor replay or a safety proof.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


CASES = ('recorded_history', 'inside_history', 'mirrored_history',
         'outside_legacy', 'inside_legacy', 'far_live', 'overshoot_history',
         'alignment_history', 'blocked_history')
TIMING_COLUMNS = ('callback_ms', 'kernel_ms')
COLORS = {'off': '#b4645b', 'on': '#3b789e', 'reference': '#d29544'}


def read(path):
    return json.loads(Path(path).read_text())


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def records(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines()
            if line.strip()]


def state_rows(path):
    rows = np.genfromtxt(path, delimiter=',', names=True, dtype=None,
                         encoding='utf-8')
    return np.atleast_1d(rows)


def compare_non_timing(first, second):
    """Compare recorded behavior without pretending elapsed CPU times match."""
    a, b = state_rows(first), state_rows(second)
    names_a = tuple(n for n in a.dtype.names if n not in TIMING_COLUMNS)
    names_b = tuple(n for n in b.dtype.names if n not in TIMING_COLUMNS)
    comparable = names_a == names_b and len(a) == len(b)
    fields = {}
    if comparable:
        for name in names_a:
            if a[name].dtype.kind in 'iufb' and b[name].dtype.kind in 'iufb':
                x, y = np.asarray(a[name], float), np.asarray(b[name], float)
                fields[name] = dict(exact_numeric_equal=bool(np.array_equal(x, y)),
                    max_absolute_difference=float(np.max(np.abs(x-y))),
                    equal_within_1e_12=bool(np.allclose(x, y, rtol=0., atol=1e-12)))
            else:
                fields[name] = dict(exact_equal=bool(np.array_equal(a[name], b[name])))
    exact = comparable and all(v.get('exact_numeric_equal', v.get('exact_equal'))
                               for v in fields.values())
    close = comparable and all(v.get('equal_within_1e_12', v.get('exact_equal'))
                               for v in fields.values())
    return dict(first=str(Path(first).resolve()), second=str(Path(second).resolve()),
        first_rows=len(a), second_rows=len(b), comparable=bool(comparable),
        ignored_columns=list(TIMING_COLUMNS), non_timing_exact_equal=bool(exact),
        non_timing_equal_within_1e_12=bool(close), fields=fields,
        scope='Recorded state, applied command, phase and stop-reason columns; '
              'not a byte-identical binary or timing-equivalence claim.')


def load_trial(path):
    meta, result = read(path/'metadata.json'), read(path/'result.json')
    rows = state_rows(path/'state.csv')
    reference = np.atleast_2d(np.loadtxt(path/'reference.csv', delimiter=',', skiprows=1))
    states = np.column_stack([rows[n] for n in ('x', 'y', 'yaw', 'v', 'w')])
    times = np.asarray(rows['t'], dtype=float)
    if result['duration_s'] > times[-1]+1e-9:
        # The runner logs a pre-step state and separately saves the true final
        # post-step state. Append that measured endpoint, not an extrapolation.
        times = np.r_[times, result['duration_s']]
        states = np.vstack((states, np.asarray(result['final_state'], float)))
    return dict(path=path, metadata=meta, result=result, rows=rows,
        reference=reference, states=states, times=times,
        body=np.asarray(meta['geometry'], dtype=float), searches=records(path/'search.jsonl'))


def snapshot(trial, t):
    index = max(0, min(len(trial['times'])-1,
                     int(np.searchsorted(trial['times'], t, side='right')-1)))
    return index, trial['states'][index]


def disk_centers(state, body):
    co, si = np.cos(state[2]), np.sin(state[2])
    return body[:, :2]@np.array([[co, si], [-si, co]])+state[:2]


def view_bounds(trials):
    points = np.vstack([trial['states'][:, :2] for trial in trials]
                       + [trials[0]['reference'][-1:, :2]])
    pad = max(float(np.max(np.linalg.norm(t['body'][:, :2], axis=1)+t['body'][:, 2]))
              for t in trials)+.12
    low, high = points.min(axis=0)-pad, points.max(axis=0)+pad
    center = .5*(low+high)
    span = max(float(np.max(high-low)), 1.)
    return (center[0]-.5*span, center[0]+.5*span,
            center[1]-.5*span, center[1]+.5*span)


def setup_axis(axis, reference, bounds, title):
    axis.plot(reference[:, 0], reference[:, 1], '--', color=COLORS['reference'],
              lw=1.3, label='REMANI reference')
    axis.plot(reference[-1, 0], reference[-1, 1], 'x', color='#333333', ms=6)
    axis.set(xlim=bounds[:2], ylim=bounds[2:], xlabel='x (m)', ylabel='y (m)', title=title)
    axis.set_aspect('equal', adjustable='box')
    axis.grid(alpha=.15)


def draw_body(axis, state, body, color, alpha=.65, fill=False):
    from matplotlib.patches import Circle
    for center, disk in zip(disk_centers(state, body), body):
        axis.add_patch(Circle(center, disk[2], fill=fill, facecolor=color,
                              edgecolor=color, lw=.9, alpha=alpha))


def make_figure(trials, out):
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(9.5, 3.8), gridspec_kw={'width_ratios': [1.15, 1.]})
    for mode, trial in zip(('off', 'on'), trials):
        distance = np.linalg.norm(trial['states'][:, :2]-trial['reference'][-1, :2], axis=1)
        times = trial['times']
        if times[-1] < 20.:
            times, distance = np.r_[times, 20.], np.r_[distance, distance[-1]]
        axes[0].plot(times, distance, color=COLORS[mode], label=mode.upper(), lw=1.8)
    axes[0].axhline(.05, color='#999999', ls=':', lw=1., label='Position tolerance')
    axes[0].set(xlim=(0., 20.), ylim=(0., None), xlabel='Simulation time (s)',
                ylabel='Distance to goal (m)', title='Recorded endpoint case')
    axes[0].grid(alpha=.2)
    axes[0].legend(frameon=False, fontsize=8)
    bounds = view_bounds(trials)
    setup_axis(axes[1], trials[0]['reference'], bounds, 'Executed motion and full projection')
    for mode, trial in zip(('off', 'on'), trials):
        path = trial['states']
        axes[1].plot(path[:, 0], path[:, 1], color=COLORS[mode], lw=1.6, label=mode.upper())
        if mode == 'on':
            _, middle = snapshot(trial, .5*trial['result']['duration_s'])
            draw_body(axes[1], middle, trial['body'], COLORS[mode], alpha=.20)
        draw_body(axes[1], path[-1], trial['body'], COLORS[mode], alpha=.7)
    axes[1].legend(frameon=False, fontsize=8, loc='upper left')
    fig.suptitle('Fixed-reference numerical test (not ROS / Gazebo / real robot)', fontsize=10)
    fig.text(.5, .02, 'Simulation time; completed trial held at its final measured state.',
             ha='center', fontsize=8, color='#555555')
    fig.tight_layout(rect=[0, .04, 1, .94])
    fig.savefig(out/'terminal_liveness.png', dpi=160)
    plt.close(fig)


def candidate_paths(trial, t, maximum=5):
    # Traces contain only stored real kernel candidates; no synthetic search.
    if t >= trial['result']['duration_s'] or not trial['searches']:
        return []
    times = [item['t'] for item in trial['searches']]
    index = int(np.searchsorted(times, t, side='right')-1)
    if index < 0 or t-times[index] > .30:
        return []
    rows = np.asarray(trial['searches'][index]['rows'], float)
    if rows.size == 0:
        return []
    ids = np.unique(rows[:, 0])
    selected = [identity for identity in ids if np.any(rows[rows[:, 0] == identity, 7] > .5)]
    feasible = [identity for identity in ids if np.all(rows[rows[:, 0] == identity, 6] > .5)]
    chosen = list(selected)
    for identity in feasible+list(ids):
        if identity not in chosen:
            chosen.append(identity)
        if len(chosen) >= maximum:
            break
    return [(rows[rows[:, 0] == identity, 2:4], identity in selected)
            for identity in chosen[:maximum]]


def make_gif(trials, out):
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter
    from matplotlib.collections import LineCollection
    from matplotlib.patches import Circle

    class FullFrameWriter(PillowWriter):
        def finish(self):
            # This host's old PillowWriter leaves RGBA transparency/disposal
            # implicit, which can produce stale text/poses in late GIF frames.
            # Explicit opaque frames and background disposal prevent that.
            frames = [frame.convert('RGB') for frame in self._frames]
            frames[0].save(self._outfile, save_all=True, append_images=frames[1:],
                           duration=100, loop=0, disposal=2, optimize=False)

    fig, axes = plt.subplots(1, 2, figsize=(8., 4.2))
    bounds, layers = view_bounds(trials), []
    for mode, trial, axis in zip(('off', 'on'), trials, axes):
        setup_axis(axis, trial['reference'], bounds, mode.upper())
        travelled, = axis.plot([], [], color=COLORS[mode], lw=1.6)
        proposals = LineCollection([], colors='#a4bdcd', linewidths=.65, alpha=.65)
        selected = LineCollection([], colors=COLORS[mode], linewidths=1., alpha=.65)
        axis.add_collection(proposals)
        axis.add_collection(selected)
        disks = []
        for disk in trial['body']:
            circle = Circle((0, 0), disk[2], facecolor=COLORS[mode], edgecolor=COLORS[mode],
                            alpha=.17, lw=1.)
            axis.add_patch(circle)
            disks.append(circle)
        heading, = axis.plot([], [], color='#333333', lw=1.2)
        label = axis.text(.03, .96, '', transform=axis.transAxes, va='top', fontsize=8)
        layers.append((travelled, proposals, selected, disks, heading, label))
    title = fig.suptitle('', fontsize=10)
    fig.text(.5, .02, 'Fixed reference | Full projected body | Stored MPPI candidates | Final pose held',
             ha='center', fontsize=8, color='#555555')
    fig.tight_layout(rect=[0, .05, 1, .92])

    def update(frame):
        t = frame/10.
        title.set_text('Numerical free-field test — simulation time %.1f s' % t)
        changed = [title]
        for trial, layer in zip(trials, layers):
            travelled, proposals, selected, disks, heading, label = layer
            index, state = snapshot(trial, t)
            travelled.set_data(trial['states'][:index+1, 0], trial['states'][:index+1, 1])
            paths = candidate_paths(trial, t)
            proposals.set_segments([path for path, chosen in paths if not chosen])
            selected.set_segments([path for path, chosen in paths if chosen])
            for circle, center in zip(disks, disk_centers(state, trial['body'])):
                circle.center = center
            end = state[:2]+.20*np.array([np.cos(state[2]), np.sin(state[2])])
            heading.set_data([state[0], end[0]], [state[1], end[1]])
            distance = float(np.linalg.norm(state[:2]-trial['reference'][-1, :2]))
            held = t >= trial['result']['duration_s']
            label.set_text('Goal distance %.3f m%s' % (distance, '\nFinal pose held' if held else ''))
            changed += [travelled, proposals, selected, heading, label]+disks
        return changed

    # GIF frame delays have 10ms resolution; 100ms is exact, unlike 125ms.
    movie = FuncAnimation(fig, update, frames=200, interval=100, blit=False, repeat=False)
    movie.save(out/'terminal_liveness.gif', writer=FullFrameWriter(fps=10), dpi=90)
    plt.close(fig)


def generate(paired, baseline, out):
    protocol = read(paired/'protocol.json')
    if tuple(protocol['cases']) != CASES or protocol['mode'] != 'paired':
        raise ValueError('Expected the predeclared nine-case OFF/ON protocol')
    if protocol['duration_s'] != 20.:
        raise ValueError('The key GIF is declared for the 20-second endpoint protocol')
    files, trial_results, trial_metadata = [], [], []
    for case in CASES:
        for mode in ('off', 'on'):
            path = paired/(case+'_'+mode)
            result, meta = read(path/'result.json'), read(path/'metadata.json')
            if result['case'] != case or bool(result['enabled']) != (mode == 'on'):
                raise ValueError('Mislabeled result: '+str(path))
            trial_results.append(dict(path=str(path.resolve()), **result))
            trial_metadata.append(meta)
            files += [path/name for name in ('result.json', 'metadata.json', 'state.csv',
                                              'reference.csv', 'search.jsonl', 'decisions.jsonl')]
    common_library = len({m['library_sha256'] for m in trial_metadata}) == 1
    common_runner = len({m['runner_sha256'] for m in trial_metadata}) == 1
    if not common_library or not common_runner:
        raise ValueError('Paired runtime/library drift')
    comparison = compare_non_timing(baseline/'recorded_history_off/state.csv',
                                    paired/'recorded_history_off/state.csv')
    legacy = {case: compare_non_timing(paired/(case+'_off')/'state.csv',
                                      paired/(case+'_on')/'state.csv')
              for case in ('outside_legacy', 'inside_legacy')}
    aggregates = {}
    for enabled in (False, True):
        group = [r for r in trial_results if bool(r['enabled']) == enabled]
        free = [r for r in group if r['case'] != 'blocked_history']
        blocked = next(r for r in group if r['case'] == 'blocked_history')
        aggregates['on' if enabled else 'off'] = dict(
            free_passed=sum(bool(r['passed']) for r in free), free_total=len(free),
            free_failed_cases=[r['case'] for r in free if not r['passed']],
            blocked_passed=bool(blocked['passed']),
            blocked_safety_rejection_valid=bool(blocked['negative_safety_rejection_valid']),
            collisions=sum(bool(r['collision']) for r in group),
            deadline_misses=sum(int(r['deadline_misses']) for r in group))
    files += [paired/'protocol.json', baseline/'recorded_history_off/state.csv',
              baseline/'recorded_history_off/result.json', baseline/'recorded_history_off/metadata.json']
    inputs = {str(p.resolve()): sha(p) for p in files}
    trials = [load_trial(paired/('recorded_history_'+mode)) for mode in ('off', 'on')]
    if not np.array_equal(trials[0]['reference'], trials[1]['reference']):
        raise ValueError('Key plot references differ')
    out.mkdir(parents=True, exist_ok=False)
    summary = dict(schema_version=1, scope=__doc__, paired=str(paired.resolve()),
        baseline=str(baseline.resolve()), predeclared_cases=list(CASES),
        common_library=common_library, common_runner=common_runner,
        baseline_vs_new_off=comparison, legacy_off_on=legacy, aggregates=aggregates,
        trials=trial_results, inputs_sha256=inputs, report_source_sha256=sha(Path(__file__)),
        visuals=dict(case='recorded_history', shared_simulation_duration_s=20.,
            gif_fps=10, playback_duration_s=20., completed_trial_display='Hold final measured pose; hide stale search',
            gif_encoding='Opaque RGB frames, 100ms, background disposal=2, optimization disabled',
            body='All original projected disks, not a point or one enlarged circle',
            search='At most five stored actual candidates, selected included; not every sample',
            trace_max_age_s=.30, figures_are_measured_wall_time=False,
            wider_regression_included=False),
        limitations=['Ideal synchronous-state free-field fixture, not original sensor replay.',
            'Original history priming has no physical advance.',
            'No ROS, native online REMANI, Gazebo or real robot is executed by this report.',
            'Kernel clearance and publication/braking validation are retained; '
            'this report does not claim the newer shared-objective implementation is used.',
            'Fixed-step simulation and host callback deadlines are distinct measurements.',
            'Finite test success is not a proof of global liveness or continuous-time safety.'])
    make_figure(trials, out)
    make_gif(trials, out)
    for path, digest in inputs.items():
        if sha(Path(path)) != digest:
            raise RuntimeError('Measurement input changed during report: '+path)
    summary['visuals_sha256'] = {name: sha(out/name) for name in
                               ('terminal_liveness.png', 'terminal_liveness.gif')}
    (out/'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2,
                                             allow_nan=False)+'\n')
    print(json.dumps(dict(out=str(out.resolve()), aggregates=aggregates,
        baseline_off_numeric_equivalent=comparison['non_timing_equal_within_1e_12']), indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--paired', type=Path, required=True)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--measurements-complete', action='store_true',
                        help='Explicit release: all measured trials have ended before rendering.')
    args = parser.parse_args()
    if not args.measurements_complete:
        parser.error('Do not render during measured trials; pass --measurements-complete after they end')
    import matplotlib
    matplotlib.use('Agg')
    generate(args.paired.resolve(), args.baseline.resolve(), args.out.resolve())


if __name__ == '__main__':
    main()
