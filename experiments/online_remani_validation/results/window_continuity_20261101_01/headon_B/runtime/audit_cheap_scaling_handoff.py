#!/usr/bin/env python3
"""Read-only chain and gear-projection audit of the six recorded SIL trials.

No simulator, native solver, controls, or policy changes. The late-stop score
reconstruction transcribes the attested current proposal/project equations;
moving-rollout 3-D collision/risk costs are explicitly omitted.
"""
import gzip
import hashlib
import json
import math
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
BATCH = HERE/'results/cheap_scaling_online_20261030_01'
BUILD = HERE/'builds/cheap_scaling_20261030_01'
TRIAL = BATCH/'midcross_cheap_seed11'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def rows(path):
    return [json.loads(line) for line in Path(path).open()]


def audit_chain(trial):
    ds = rows(trial/'dual_timeline.jsonl')
    opt = [v for v in rows(trial/'solver_trace.jsonl')
           if v.get('event') == 'end' and v.get('stage') == 'optimizer']
    cycles = rows(trial/'state_mppi_cycles.jsonl')
    refs = rows(trial/'reference_events.jsonl')
    failures = rows(trial/'failure_trace.jsonl')
    requests = []
    for begin in (r for r in ds if r['event'] == 'request_begin'):
        rid = begin['request_id']
        events = [r for r in ds if r.get('request_id') == rid]
        publish = next((r for r in events if r['event'] == 'reference_publish'), None)
        manager = next((r for r in events if r['event'] == 'manager_result'), None)
        reject = [r.get('reason') for r in events if r['event'] == 'acceptance_reject']
        r = dict(request_id=rid, manager_success=manager['success'] if manager else None,
                 acceptance_reject=reject, published=publish is not None)
        if publish:
            # guide_id is reused; join by this publication's next complete
            # receipt, then by its revision, rather than by guide_id alone.
            receipt = min((z for z in refs if z['mono'] >= publish['mono']), key=lambda z: z['mono'])
            rev = receipt['revision']
            first = next((z for z in cycles if z.get('guide_revision') == rev and
                          z.get('first_kernel_cycle_for_reference')), None)
            valid = next((z for z in cycles if z.get('guide_revision') == rev and
                          z.get('first_validated_command_cycle_for_reference')), None)
            r.update(revision=rev, request_to_publish_s=publish['mono']-begin['mono'],
                published_reference_age_s=publish['sim']-publish['guide_start'],
                publish_to_receipt_s=receipt['mono']-publish['mono'],
                publish_to_first_kernel_s=None if first is None else first['online_step_entry_monotonic']-publish['mono'],
                publish_to_first_validated_s=None if valid is None else valid['command_publish_after_monotonic']-publish['mono'])
        requests.append(r)
    return dict(name=trial.name, result=json.loads((trial/'result.json').read_text()),
        optimizer_count=len(opt), native_pass_count=sum(r['success'] for r in opt),
        publish_count=sum(r['event'] == 'reference_publish' for r in ds),
        first_kernel_count=sum(bool(r.get('first_kernel_cycle_for_reference')) for r in cycles),
        first_validated_count=sum(bool(r.get('first_validated_command_cycle_for_reference')) for r in cycles),
        input_identities=[dict(input_hash=r['input_hash'], first_cost=r['first_cost'],
                              first_gradient_hash=r['first_gradient_hash']) for r in failures],
        optimizers=[dict(optimizer_id=r['id'], budget_s=r['budget_s'], success=r['success'],
            prepare_ms=r.get('cheap_scaling_prepare_ms'), prepare_plus_solve_ms=r.get('lbfgs_wall_ms'),
            complete_evaluations=r['iterations'], force_stop=r['force_stop'],
            pin_owned=r.get('solver_overlay_pin_owned'),
            entry_exit_overlay_equal=r.get('solver_overlay_entry_key') == r.get('solver_overlay_return_key')) for r in opt],
        requests=requests)


def reconstruct(ref, initial, config, horizon):
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(ref[:, :2], axis=0), axis=1))]
    wrap = lambda x: math.atan2(math.sin(x), math.cos(x))

    def bound(u):
        v, w = u
        wheels = [v-.5*config['track']*w, v+.5*config['track']*w]
        k = max(1., abs(v)/config['vmax'], abs(w)/config['wmax'],
                abs(wheels[0])/config['wheel_v'], abs(wheels[1])/config['wheel_v'])
        return np.asarray(u)/k

    def step(state, control, duration):
        s, u = state.copy(), bound(control)
        n = max(1, math.ceil(duration/.01-1e-10))
        h = duration/n
        for unused in range(n):
            a = (u-s[3:])/.12
            wa = [a[0]-.5*config['track']*a[1], a[0]+.5*config['track']*a[1]]
            k = max(1., abs(a[0])/config['accel'], abs(a[1])/config['angular_accel'],
                    abs(wa[0])/config['wheel_a'], abs(wa[1])/config['wheel_a'])
            a /= k
            vn, wn = s[3]+h*a[0], s[4]+h*a[1]
            vm, wm = .5*(s[3]+vn), .5*(s[4]+wn)
            heading = s[2]+.5*h*wm
            s[0] += h*vm*math.cos(heading)
            s[1] += h*vm*math.sin(heading)
            s[2], s[3], s[4] = wrap(s[2]+h*wm), vn, wn
        return s

    def project(state, hint=-1):
        lo = 0 if hint < 0 else max(0, hint-8)
        hi = len(ref)-1 if hint < 0 else min(len(ref)-1, hint+45)
        best, fraction = [0, 1e9, 0., 0., 0.], 0.
        for i in range(lo, hi):
            d = ref[i+1, :2]-ref[i, :2]
            length = arc[i+1]-arc[i]
            if length < 1e-9:
                continue
            f = np.clip((state[:2]-ref[i, :2])@d/length**2, 0., 1.)
            delta = state[:2]-(ref[i, :2]+f*d)
            distance = delta@delta
            if distance < best[1]:
                best, fraction = [i, distance, arc[i]+f*length, 0., 0.], f
        i = best[0]
        return [i, math.sqrt(best[1]), best[2],
                wrap(ref[i, 2]+fraction*wrap(ref[i+1, 2]-ref[i, 2])),
                (1-fraction)*ref[i, 3]+fraction*ref[i+1, 3]]

    def direction(index):
        for j in list(range(index, min(len(ref), index+12)))+list(range(index, -1, -1)):
            if abs(ref[j, 3]) > .005:
                return 1 if ref[j, 3] > 0. else -1
        return 1

    def guide(state, hint):
        q = project(state, hint)
        hint, sign = q[0], direction(q[0])
        goal, remaining = np.linalg.norm(ref[-1, :2]-state[:2]), arc[-1]-q[2]
        target, lookahead = hint, .20+.35*abs(state[3])
        while target+1 < len(ref) and arc[target] < q[2]+lookahead:
            if abs(ref[target, 3]) > .005 and direction(target) != sign:
                break
            target += 1
        delta = ref[target, :2]-state[:2]
        travel = state[2]+(math.pi if sign < 0 else 0.)
        heading = wrap(math.atan2(delta[1], delta[0])-travel)
        if abs(heading) > 1.2:
            return bound([0., 1.8*heading]), hint
        lateral = -math.sin(travel)*delta[0]+math.cos(travel)*delta[1]
        speed = max(abs(q[4]), abs(ref[target, 3]))
        speed = min(speed, .85*config['wheel_v'], .9*max(goal, remaining))
        speed *= max(.2, math.cos(heading))
        return bound([sign*speed, 2*speed*lateral/max(.015, delta@delta)]), hint

    result = []
    for scale in (1., .7, .35, 0.):
        state, hint = initial.copy(), project(initial)[0]
        controls, gears = [], []
        for unused in range(horizon):
            u, hint = guide(state, hint)
            gears.append(direction(hint))
            u = bound(scale*u)
            controls.append(u)
            state = step(state, u, .1)
        state, q, cost, previous = initial.copy(), project(initial), 0., initial[3:].copy()
        start, indices = q[2], []
        for u in controls:
            old = q[2]
            state = step(step(state, u, .05), u, .05)
            q = project(state, q[0])
            indices.append(q[0])
            rate, goal = (q[2]-old)/.1, np.linalg.norm(ref[-1, :2]-state[:2])
            wanted = min(abs(q[4]), .9*goal)
            deficit = np.clip((wanted-rate)/wanted, 0., 2.)**2 if wanted > .08 else 0.
            target = q[3]
            if q[1] > .08 or goal < .25:
                look = q[0]
                while look+1 < len(ref) and arc[look] < q[2]+.35:
                    look += 1
                delta = ref[look, :2]-state[:2]
                target = wrap(math.atan2(delta[1], delta[0])+(math.pi if direction(q[0]) < 0 else 0.))
            cost += .1*(5*q[1]**2+.6*wrap(state[2]-target)**2+1.5*deficit+
                        2*max(0., -direction(q[0])*state[3])**2)
            du = u-previous
            cost += .02*(du@du)
            previous = u
        cost += 12*q[1]**2-12*(q[2]-start)
        result.append(dict(following_scale=scale, reconstructed_cost_without_3d_terms=float(cost),
            progress_m=float(q[2]-start), first_control=controls[0].tolist(),
            guide_gears=gears, rollout_projection_indices=indices, final_state=state.tolist()))
    return dict(initial_projection=project(initial), path_length_m=float(arc[-1]), candidates=result)


def main():
    manifest = json.loads((BUILD/'build_manifest.json').read_text())
    sources = {}
    for name in ('online_kernel.hpp', 'mppi_v12.hpp'):
        path = BUILD/'source'/name
        sources[name] = sha(path)
        if sources[name] != manifest['isolated_source_sha256'][name]:
            raise ValueError('Current child source differs from compiled manifest: '+name)
    library = BUILD/'build/libonline_mppi.so'
    if sha(library) != manifest['mppi_library_sha256']:
        raise ValueError('Native MPPI library hash differs from compiled manifest')
    refs = rows(TRIAL/'consumed_references.jsonl')
    refrow = refs[-1]
    reference = np.asarray(refrow['reference'])
    if hashlib.sha256(np.asarray(reference, dtype='<f8').tobytes()).hexdigest() != refrow['sha256']:
        raise ValueError('Consumed reference numeric hash mismatch')
    result = json.loads((TRIAL/'result.json').read_text())
    state = np.asarray(result['final_state'])
    diagnostics = json.load(gzip.open(TRIAL/'mppi_diagnostics.json.gz', 'rt'))
    last = diagnostics[-1]
    decision = last['local_phase']['decision']
    horizon = int(round(decision[2]/.1))
    config = json.loads((TRIAL/'limits.json').read_text())
    rebuilt = reconstruct(reference, state, config, horizon)
    recorded_cost = decision[9]
    zero_cost = rebuilt['candidates'][-1]['reconstructed_cost_without_3d_terms']
    if abs(zero_cost-recorded_cost) > 1e-12:
        raise ValueError('Reconstructed stop merit does not match recorded decision')
    changes = (np.flatnonzero(np.signbit(reference[1:, 3]) != np.signbit(reference[:-1, 3]))+1).tolist()
    trials = [audit_chain(BATCH/name) for name in (
        'empty_unscaled_seed11', 'empty_cheap_seed11', 'static_cheap_seed11',
        'static_unscaled_seed11', 'midcross_unscaled_seed11', 'midcross_cheap_seed11')]
    report = dict(schema='cheap_scaling_handoff_audit_v1', current_child_source_sha256=sources,
        child_manifest_sha256=sha(BUILD/'build_manifest.json'), native_mppi_library_sha256=sha(library),
        audit_script_sha256=sha(__file__), trial_sources_sha256={name:sha(TRIAL/name) for name in
            ('result.json', 'consumed_references.jsonl', 'mppi_diagnostics.json.gz', 'limits.json')},
        final_state=state.tolist(), reference_sha256=refrow['sha256'], reference_points=len(reference),
        reference_tail=reference[-1].tolist(), final_goal_distance_m=float(np.linalg.norm(reference[-1, :2]-state[:2])),
        sign_change_indices=changes, recorded_last_decision=decision,
        recorded_last_reason=last['reason'], recorded_last_obstacles=last['observed_obstacles'],
        zero_score_absolute_error=abs(zero_cost-recorded_cost), reconstruction=rebuilt, trials=trials,
        limitations=['The final simulator truth state is used; compact cycle records do not export the exact final solver state.',
            'Moving-rollout native 3-D collision soft cost and closing-risk cost are omitted. This is a score/projection reconstruction, not a full native-kernel replay.',
            'No intervention was applied. No strategy, solver, acceptance gate, or controls were changed.',
            'The four rows are following(scale) proposals. A zero warm sequence can be an additional duplicate candidate.',
            'Fresh online refresh inputs differ between groups; only the reported identical initial hashes support a same-input comparison.'])
    out=BATCH/'report'
    out.mkdir(exist_ok=True)
    (out/'handoff_audit.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    text = '''独立交付与换挡停滞审计

1. 空旷初始请求确实是同一输入：两组input、first cost、first gradient哈希相同。
   原组66次完整评估、准备加求解约410毫秒；缩放组111次、约676毫秒，其中准备仅14毫秒。
   所以本批普通输入没有加速，退化主要不只是准备开销。不能只报告冻结失败输入改善。

2. 静态两组后端均2/2通过，均发布2条参考，2条都被MPPI实际计算和验证采用，无最终接受拒绝。
   刷新请求墙钟：缩放约842毫秒，原组约691毫秒；这两个在线刷新输入不同，不能当严格同输入因果比较。
   发布时参考起点时间已经过去：缩放约430毫秒，原组约286毫秒。没有late_moving_handoff拒收不等于参考没有老化。

3. 动态原组后端3/5通过，最终交付3条；缩放组后端3/3通过，但第二次请求被最新预测碰撞检查拒收，最终交付2条。
   两组已交付的参考均有真实接收、首次MPPI计算和首次验证指令记录。不能把manager成功当作最终交付成功。
   本批未出现late_moving_handoff拒收。缩放组最后一条发布时，guide_start已过去约1.659秒。

4. 缩放组最终超时的直接停滞发生在MPPI执行换挡点，而不是等待REMANI求解。
   最后状态距目标约36.36厘米，却距参考倒车转前进的重复点仅约0.18毫米。
   参考903点，索引817/818坐标相同，速度从-0.05变+0.05；最终MPPI投影索引818，误差约0.169毫米。
   末尾无动态障碍，reason持续ok，验证通过，选择following(scale=0)停车，未再触发新请求。

5. 按当前编译子版本源码重建的机制：
   前进候选初始投影818，下一步跳回815，第三步引导方向又变成倒车；之后投影卡在816附近。
   四个following候选的重建成本约0.68930、0.31130、0.06026、0.04399199，停车最便宜。
   停车成本与实际最后decision成本相符到浮点精度。
   移动候选省去了原生三维软碰撞和闭合风险项，因此这里是公式与投影机制重建，不是完整原生回放或干预验证。

已确认：普通输入缩放可能退化；原生后端通过后仍可能被最新预测拒收；新参考真实采用后，现有MPPI仍会在重叠换挡段停住。
尚未完成：没有在本轮修改换挡策略或做干预，不能说换挡问题已解决，也不能把整条动态超时全归因于轨迹交付。
下一步最小验证：固定这个停点、参考和感知输入，只比较原投影与按当前换挡段约束的投影，先看是否能稳定跨过换挡点。

本审计仅读取已有结果与源码，没有新开仿真、没有发布控制、没有修改求解和验收规则。
完整来源哈希、请求级计数、参考采用时钟、候选分数和投影序列见handoff_audit.json。
'''
    (out/'独立交付与换挡停滞审计.txt').write_text(text)
    print(out/'handoff_audit.json')
    print(out/'独立交付与换挡停滞审计.txt')


if __name__ == '__main__':
    main()
