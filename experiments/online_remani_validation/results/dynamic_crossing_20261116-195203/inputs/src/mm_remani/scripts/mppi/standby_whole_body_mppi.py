"""Explicit standby extension of the experimental whole-body MPPI kernel.

This is a new, separately named capability.  It does not grant ROS execution
authority or create a navigation goal.  Its caller supplies a fixed soft
standby reference (x, y, yaw, signed speed) and measured collision snapshots.
All eight command coordinates remain available, including at zero goal error.
Only the inherited native sphere model and snapshot.query authorize motion;
no scenario description, obstacle truth or scripted avoidance time is read.
"""

import numpy as np

from active_arm_mppi import WholeBodyMPPI, wrap


DEFAULT_CONFIG = {
    'lambda_temperature': .15,
    'noise_sigma': [.012, .08, .035, .035, .035, .035, .035, .035],
    'rho': .75,
    'weighted_accept_slack': .002,
    'base_modes': [
        ['hold', 0., 0.], ['forward', .08, 0.], ['reverse', -.08, 0.],
        ['turn_left', 0., .24], ['turn_right', 0., -.24],
        ['curve_left', .06, .18], ['curve_right', .06, -.18],
    ],
    'weights': {
        'anchor_xy': 15., 'anchor_yaw': 2.,
        'base_command_v': 3., 'base_command_w': .20,
        'arm_reference': .35, 'arm_command': .015,
        'command_change': .025,
        'terminal_xy': 20., 'terminal_yaw': 3.,
        'terminal_arm_reference': .50,
    },
}


class StandbyWholeBodyMPPI(WholeBodyMPPI):
    """Whole-body reactive standby sampling; same plan/evaluate API as parent."""

    def __init__(self, *args, config=None, **kwargs):
        super().__init__(*args, **kwargs)
        supplied = {} if config is None else dict(config)
        unknown = set(supplied)-set(DEFAULT_CONFIG)
        if unknown:
            raise ValueError('unknown standby config: '+','.join(sorted(unknown)))
        self.standby_config = dict(DEFAULT_CONFIG)
        self.standby_config.update(supplied)
        weights = dict(DEFAULT_CONFIG['weights'])
        custom_weights = supplied.get('weights', {})
        if set(custom_weights)-set(weights):
            raise ValueError('unknown standby weight')
        weights.update(custom_weights)
        self.standby_config['weights'] = weights
        self.weights = weights
        self.temperature = float(self.standby_config['lambda_temperature'])
        self.noise_sigma = np.asarray(self.standby_config['noise_sigma'], dtype=float)
        self.rho = float(self.standby_config['rho'])
        self.weighted_accept_slack = float(self.standby_config['weighted_accept_slack'])
        if (not np.isfinite(self.temperature) or self.temperature <= 0. or
                self.noise_sigma.shape != (8,) or not np.isfinite(self.noise_sigma).all() or
                np.any(self.noise_sigma <= 0.) or not 0. <= self.rho < 1. or
                not np.isfinite(self.weighted_accept_slack) or self.weighted_accept_slack < 0. or
                any(not np.isfinite(v) or v < 0. for v in weights.values())):
            raise ValueError('invalid standby noise, temperature or weights')
        self.base_modes = []
        for mode in self.standby_config['base_modes']:
            if len(mode) != 3 or not isinstance(mode[0], str):
                raise ValueError('invalid standby base mode')
            command = np.asarray(mode[1:], dtype=float)
            if command.shape != (2,) or not np.isfinite(command).all():
                raise ValueError('invalid standby base command')
            self.base_modes.append((mode[0], command))
        if not self.base_modes or not any(np.linalg.norm(u) > 0. for _, u in self.base_modes):
            raise ValueError('standby base candidates must include motion')
        if self.random_samples < 1:
            raise ValueError('standby MPPI requires random samples in all eight coordinates')
        self.last_diagnostics = None

    @staticmethod
    def anchor(reference):
        r = np.asarray(reference, dtype=float)
        if r.ndim != 2 or r.shape[1] != 4 or not len(r) or not np.isfinite(r).all():
            raise ValueError('standby reference must be finite N x 4')
        return r[-1, :3]

    def guide(self, state, reference, speed_scale=1., angle_offset=0.):
        """Soft recovery guide for logging; proposals retain their own freedom."""
        anchor = self.anchor(reference)
        delta = anchor[:2]-state[:2]
        along = delta @ np.array([np.cos(state[2]), np.sin(state[2])])
        side = delta @ np.array([-np.sin(state[2]), np.cos(state[2])])
        return np.array([np.clip(.6*along, -.08, .08)*speed_scale,
                         np.clip(1.5*wrap(anchor[2]-state[2])+.8*side+angle_offset, -.24, .24)])

    def proposals(self, state, reference, arm_reference, base_modes=None):
        """Cross arm targets with real base velocities, even exactly at anchor."""
        self.anchor(reference)
        targets = [('reference', np.asarray(arm_reference)), ('current', state[5:11])]
        targets.extend(('posture_'+str(i), target) for i, target in enumerate(self.postures))
        unique = []
        for label, target in targets:
            if np.asarray(target).shape != (6,) or not np.isfinite(target).all():
                raise ValueError('invalid standby arm target')
            if not any(np.max(np.abs(target-other)) < 1e-7 for _, other in unique):
                unique.append((label, np.clip(target, self.qmin, self.qmax)))
        modes = self.base_modes if base_modes is None else base_modes
        labels = [(arm_name, mode_name) for arm_name, _ in unique for mode_name, _ in modes]
        target_rows = np.asarray([q for _, q in unique for _ in modes])
        base_rows = np.asarray([u for _ in unique for _, u in modes])
        z = np.tile(state, (len(labels), 1))
        controls = np.empty((len(labels), self.H, 8))
        for k in range(self.H):
            # A finite evasive action followed by soft anchor recovery gives
            # each candidate a full horizon, without enforcing base equality.
            if k < max(1, int(.65*self.H)):
                ub = base_rows
            else:
                # Same soft guide for all candidates, evaluated as one batch.
                anchor = self.anchor(reference)
                delta = anchor[:2]-z[:, :2]
                cosine, sine = np.cos(z[:, 2]), np.sin(z[:, 2])
                along = delta[:, 0]*cosine+delta[:, 1]*sine
                side = -delta[:, 0]*sine+delta[:, 1]*cosine
                ub = np.column_stack((np.clip(.6*along, -.08, .08),
                    np.clip(1.5*wrap(anchor[2]-z[:, 2])+.8*side, -.24, .24)))
            ua = np.clip(1.8*(target_rows-z[:, 5:11])-.2*z[:, 11:17], -self.vmax, self.vmax)
            if not self.enabled:
                ua[:] = 0.
            controls[:, k] = np.c_[ub, ua]
            z = self.advance(z, controls[:, k])
        if self.previous is not None:
            controls = np.concatenate([controls, np.r_[self.previous[1:], self.previous[-1:]][None]])
            labels.append(('previous', 'previous'))
        return controls, labels

    def evaluate(self, state, controls, reference, arm_reference, snapshot, now):
        """Use the original rollout/geometry acceptance with a standby cost."""
        controls = np.asarray(controls, dtype=float)
        K = len(controls)
        states = np.empty((K, self.H+1, 17)); states[:, 0] = state
        dense = []; z = states[:, 0].copy(); sample_dt = self.dt/2
        for k in range(self.H):
            # A position command is held for the whole control interval, not
            # regenerated from each integration half-step.
            target = self.position_target(z, controls[:, k]) if self.arm_position_servo else None
            for _ in range(2):
                z = self.advance(z, controls[:, k], sample_dt, arm_target=target); dense.append(z)
            states[:, k+1] = z
        tail = []; z = states[:, -1].copy()
        tail_steps = np.full(K, 40, dtype=int)
        stopped = np.zeros(K, dtype=bool)
        for tail_index in range(40):
            if self.arm_position_servo:
                z = self.advance_position_brake(z, sample_dt)
            else:
                z = self.advance(z, np.zeros((K, 8)), sample_dt)
            tail.append(z)
            quiet = ((np.max(np.abs(z[:, 3:5]), axis=1) < .003) &
                     (np.max(np.abs(z[:, 11:17]), axis=1) < .003))
            tail_steps[quiet & ~stopped] = tail_index+1
            stopped |= quiet
            if np.all(stopped):
                break
        full = np.stack([states[:, 0]]+dense+tail, axis=1)
        L = full.shape[1]; flat = full.reshape((-1, 17))
        # Match evaluating each candidate alone, including its first stopped
        # sample. Other candidates' longer braking tails are batch padding.
        checked = np.arange(L)[None, :] <= 2*self.H+tail_steps[:, None]
        times = np.broadcast_to(now+sample_dt*np.arange(L), (K, L))
        flat = full[checked]
        spheres = self.spheres(flat)
        external, obstacle_cost = snapshot.query(spheres, times[checked])
        # Scatter only checked samples; padding cannot affect a candidate's
        # acceptance, diagnostics, or the horizon-only obstacle cost below.
        external_rows = np.full((K, L), np.inf)
        external_rows[checked] = external
        self.last_external_gaps = np.min(external_rows, axis=1)
        configuration = self.configuration_gap(spheres, flat[:, 5:11])
        configuration_rows = np.full((K, L), np.inf)
        configuration_rows[checked] = configuration
        gaps = np.minimum(external_rows, configuration_rows)
        bounds = np.ones((K, L), dtype=bool)
        bounds[checked] = np.all((flat[:, 5:11] >= self.qmin-1e-9) &
                                (flat[:, 5:11] <= self.qmax+1e-9), axis=1)
        obstacle_rows = np.zeros((K, L))
        obstacle_rows[checked] = obstacle_cost
        obstacle_cost = obstacle_rows
        valid = stopped & np.all((gaps >= 0.) & bounds, axis=1)
        external_violation = (external_rows < 0.) & checked
        # Compact summaries of the existing checks, including the braking tail.
        # Terminal joint deltas describe the H-step plan, before that tail.
        self.last_evaluation_diagnostics = {
            'configuration_gaps': np.min(configuration_rows, axis=1).tolist(),
            'first_external_violation_s': [float(sample_dt*np.argmax(row)) if row.any() else None
                                           for row in external_violation],
            'braking_tail_s': (sample_dt*tail_steps).tolist(),
            'checked_duration_s': (sample_dt*(2*self.H+tail_steps)).tolist(),
            'terminal_arm_delta': (states[:, -1, 5:11]-state[5:11]).tolist(),
            'bounds_valid': np.all(bounds, axis=1).tolist(),
            'braking_stopped': stopped.tolist(),
        }
        anchor = self.anchor(reference); z = states[:, 1:]; w = self.weights
        xy_error = np.sum((z[:, :, :2]-anchor[:2])**2, axis=-1)
        yaw_error = wrap(z[:, :, 2]-anchor[2])**2
        arm_error = np.mean(((z[:, :, 5:11]-arm_reference)/(self.qmax-self.qmin))**2, axis=-1)
        initial = np.tile(np.r_[state[3:5], state[11:17]], (K, 1))
        du = np.diff(np.concatenate([initial[:, None, :], controls], axis=1), axis=1)
        soft = obstacle_cost.reshape((K, L))[:, 1:1+2*self.H].reshape((K, self.H, 2)).mean(axis=2)
        cost = self.dt*np.sum(soft+w['anchor_xy']*xy_error+w['anchor_yaw']*yaw_error+
            w['base_command_v']*controls[:, :, 0]**2+w['base_command_w']*controls[:, :, 1]**2+
            w['arm_reference']*arm_error+w['arm_command']*np.sum(controls[:, :, 2:]**2, axis=-1)+
            w['command_change']*np.sum(du**2, axis=-1), axis=1)
        cost += w['terminal_xy']*xy_error[:, -1]+w['terminal_yaw']*yaw_error[:, -1]
        cost += w['terminal_arm_reference']*arm_error[:, -1]
        return cost, valid, np.min(gaps, axis=1), states

    def plan(self, state, reference, arm_reference, snapshot, now):
        us, labels = self.proposals(state, reference, arm_reference)
        cost, valid, gaps, states = self.evaluate(state, us, reference, arm_reference, snapshot, now)
        self.record_visual_rollouts(states, valid, reset=True)
        external = self.last_external_gaps.copy()
        structured_diagnostics = self.last_evaluation_diagnostics
        ids = np.flatnonzero(valid)
        # Even if every structured candidate fails, retain its least-violating
        # member as a stochastic seed; sampled controls receive the same checks.
        best = int(ids[np.argmin(cost[ids])]) if len(ids) else int(np.argmax(gaps))
        noise = np.zeros((self.random_samples, self.H, 8))
        for k in range(self.H):
            noise[:, k] = (self.rho*noise[:, k-1] if k else 0.) + self.rng.normal(
                size=(self.random_samples, 8))*self.noise_sigma
        if not self.enabled:
            noise[:, :, 2:] = 0.
        extras = us[best][None]+noise
        ec, ev, eg, es = self.evaluate(state, extras, reference, arm_reference, snapshot, now)
        self.record_visual_rollouts(es, ev)
        random_external = self.last_external_gaps.copy()
        info = {
            'mode': 'standby_extension', 'intervention': bool(not valid[0] or external[0] < .12),
            'nominal_external_min_gap': float(external[0]),
            'structured_candidates': len(us), 'structured_labels': [list(x) for x in labels],
            'structured_valid': valid.tolist(), 'structured_costs': cost.tolist(),
            'structured_external_gaps': external.tolist(),
            **{'structured_'+key: value for key, value in structured_diagnostics.items()},
            'structured_first_base_commands': us[:, 0, :2].tolist(),
            'random_samples': self.random_samples, 'sample_dimensions': 8,
            'noise_sigma': self.noise_sigma.tolist(), 'noise_rho': self.rho,
            'lambda_temperature': self.temperature,
            'noise_rms_per_coordinate': np.sqrt(np.mean(noise**2, axis=(0, 1))).tolist(),
            'base_noise_rms': np.sqrt(np.mean(noise[:, :, :2]**2, axis=(0, 1))).tolist(),
            'random_valid': ev.tolist(), 'feasible': int(valid.sum()+ev.sum()),
            'candidates': int(len(us)+len(extras)+1),
            'standby_anchor': self.anchor(reference).tolist(),
        }
        # Average only samples in the selected maneuver family. Opposite
        # turn/arm strategies never cancel through unconditional averaging.
        same = [i for i in ids if labels[i] == labels[best]]
        pool = np.concatenate([us[same], extras[ev]])
        pc = np.r_[cost[same], ec[ev]]
        if not len(pool):
            info.update(selection='none', weights=[], min_gap=float(max(gaps.max(), eg.max())))
            self.last_diagnostics = info
            return None, info
        weights = np.exp(-(pc-pc.min())/self.temperature); weights /= weights.sum()
        mean = np.sum(pool*weights[:, None, None], axis=0)
        mc, mv, mg, ms = self.evaluate(state, mean[None], reference, arm_reference, snapshot, now)
        self.record_visual_rollouts(ms, mv)
        mean_external = float(self.last_external_gaps[0])
        options = []
        if len(ids):
            options.append((float(cost[best]), us[best], states[best], float(gaps[best]),
                            'structured', list(labels[best]), float(external[best])))
        if np.any(ev):
            rid = int(np.flatnonzero(ev)[np.argmin(ec[ev])])
            options.append((float(ec[rid]), extras[rid], es[rid], float(eg[rid]),
                            'random', list(labels[best]), float(random_external[rid])))
        selected = min(options, key=lambda entry: entry[0])
        if mv[0] and mc[0] <= selected[0]+self.weighted_accept_slack:
            selected = (float(mc[0]), mean, ms[0], float(mg[0]),
                        'exponential_mean', list(labels[best]), mean_external)
        score, controls, prediction, gap, source, family, external_gap = selected
        self.previous = controls.copy()
        delta = prediction[-1, :3]-state[:3]
        info.update(selection=source, selected_family=family, weights=weights.tolist(),
            weighted_pool_costs=pc.tolist(), weighted_mean_valid=bool(mv[0]), cost=score,
            min_gap=gap, external_min_gap=external_gap,
            arm_motion=float(np.linalg.norm(prediction[-1, 5:11]-state[5:11])),
            selected_first_base_command=controls[0, :2].tolist(),
            selected_base_displacement_m=float(np.linalg.norm(delta[:2])),
            selected_base_yaw_change_rad=float(wrap(delta[2])))
        self.last_diagnostics = info
        return (controls, prediction), info
