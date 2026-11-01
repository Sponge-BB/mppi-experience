#!/usr/bin/env python3
"""Read-only crosschecks and concise final presentation of completed trials."""
import argparse
import json
import math
from pathlib import Path
import re
import numpy as np
import yaml

from audit_delivery_iterates import canonical_records, matrix, vector_hash
from build_budget_scaling import check as check_build
from report_domain_recovery import assert_plot_quiet, digest
from run_dual_timescale import verify_production

HERE=Path(__file__).resolve().parent


def diagonal(capture, size):
    times=[matrix(t).ravel() for t in capture['initial_times']]
    dim=capture['initial_states'][0]['rows']
    ref=max(max(t) for t in times)
    floor=math.sqrt(np.finfo(float).eps)
    def scale(left,right):
        a=7*(math.log(ref)-math.log(left));b=7*(math.log(ref)-math.log(right))
        m=max(a,b)
        return max(floor,min(1.,math.exp(.5*(math.log(2)-m-math.log(math.exp(a-m)+math.exp(b-m))))))
    result=np.ones(size);offset=0
    for ts in times:
        for i in range(len(ts)-1):
            result[offset+dim*i:offset+dim*i+2]=scale(ts[i],ts[i+1])
        offset+=dim*(len(ts)-1)
    offset+=sum(len(t) for t in times)
    for i in range(len(times)-1):
        result[offset+dim*i:offset+dim*i+2]=scale(times[i][-1],times[i+1][0])
    return result


def verify(report, expected_iterations):
    if report.get('evidence_valid') is not True:
        raise ValueError('Incomplete/invalid formal report')
    checked=[]
    for trial in report['trials']:
        path=Path(trial['source'])
        for name,h in trial['input_sha256'].items():
            if digest(path/name)!=h:raise ValueError('Recorded evidence drift: '+str(path/name))
        params=yaml.safe_load((path/'parameters.yaml').read_text())
        if params['/remani_planner_node/optimization/max_iterations']!=expected_iterations:
            raise ValueError('Unexpected loaded iteration cap')
        rows,_=canonical_records(path)
        capture=next(r for r in rows if r.get('kind')=='capture')
        initial=[r for r in rows if r.get('kind')=='canonical_candidate' and r.get('stage')=='initial']
        signatures={(vector_hash(r['x']),vector_hash(r['g']),r['cost']) for r in initial}
        if len(initial)!=5 or len(signatures)!=1:raise ValueError('Initial canonical identity mismatch')
        starts=[r for r in rows if r.get('kind')=='coordinate_scaling_start']
        if len(starts)!=1:raise ValueError('Unexpected scaling activation count')
        actual=np.array(starts[0]['diagonal']);expected=diagonal(capture,len(initial[0]['x']))
        if actual.shape!=expected.shape or not np.allclose(actual,expected,rtol=1e-12,atol=0.):
            raise ValueError('Logged scaling differs from fixed initial-time recipe')
        checked.append(dict(trial=str(path),max_iterations=expected_iterations,
            first_xfg_equal=True,diagonal_verified=True,scaled_variables=int(sum(actual<1)),
            minimum_diagonal=float(min(actual)),all_final_markers=[p['primary']['marker'] for p in trial['phases']]))
    return checked


def main(out):
    if out.exists():raise FileExistsError(out)
    main_path=HERE/'results/budget_scaling_report_20261027_01/summary.json'
    cap_path=HERE/'results/budget_iteration_cap_report_20261027_01/summary.json'
    main_report=json.loads(main_path.read_text());cap=json.loads(cap_path.read_text())
    build=Path(main_report['build']);manifest=check_build(build)
    production=verify_production(build)
    checks=verify(main_report,300)+verify(cap,1200)
    all_trials=main_report['trials']+cap['trials']
    assert_plot_quiet(dict(trials=all_trials))
    text=(build/'source/domain_lbfgs.hpp').read_text()
    enum=text[text.index('LBFGSERR_UNKNOWNERROR = -1024'):text.index('LBFGSERR_MAXIMUMITERATION,')+len('LBFGSERR_MAXIMUMITERATION,')]
    names=re.findall(r'^\s*(LBFGSERR_\w+)\b',enum,re.M)
    code=-1024+names.index('LBFGSERR_MAXIMUMITERATION')
    if code!=-1008:raise ValueError('Unexpected native result-code mapping')
    out.mkdir(parents=True,exist_ok=False)
    summary=dict(evidence_valid=True,source_sha256=digest(Path(__file__)),
        main_report_sha256=digest(main_path),cap_report_sha256=digest(cap_path),
        build_manifest_sha256=digest(build/'build_manifest.json'),production_files_verified=len(production),
        observed_production_inventory_entries=len(manifest.get('production_sources_sha256',{})),
        process_and_port_quiet=True,maximum_iteration_result_code=code,trials=checks)
    (out/'完整性核验.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    lines=['预算与轨迹交付实验简要结论（2026-10-27）','',
        '1. 第一批：4个新固定输入，5组对照，共20次求解。0.6秒、3秒、3秒加坐标缩放，最终原生+完整投影双过并按时返回均为0/4。',
        '2. 加时间并非没有作用：两个输入在3秒内产生原生合格候选；但完整投影仍不合格。尺度组这两个末候选分别还欠约7.7/8.3毫米安全余量，不能计作可交付。',
        '3. 另两个输入的3秒臂在300次已接受迭代处结束，状态-1008表示达到迭代上限，不是线搜索错误。其缩放D全1，不能据此评价空间预条件效果。',
        '4. 部分3秒尺度组中，停止资格检查耗时约1.92/1.96秒。先做原生全轨迹验收，再发现投影不合格，造成了明显的无效检查开销。',
        '5. 机制补测：两个新输入将迭代上限提高到1200，全部5组最终合格仍为0/2。补测不是旧输入精确重放；是否实际跨过300次须看下表，不能把配置提高等同于因果收益验证。','']
    for trial in cap['trials']:
        for p in trial['phases'][2:4]:
            n=p['iterate_audit']['solve_summary']['accepted_total']
            lines.append('%s / %s：accepted=%d，%s，%.1fms。' % (trial['trial'],p['phase'],n,p['solver_reason'],p['decision_elapsed_s']*1000))
    lines += ['',
        '当前判断：交付率尚无提升；不能归结为“只差0.6秒预算”或“轮加速度限制太严”。原生轮约束、完整投影合格性，以及验收本身的计算开销需要分开处理。',
        '下一步优先：让搜索/初始MINCO曲线与最终投影模型一致，并把便宜的投影筛查放到昂贵原生验收之前；通过率仍以全部原安全门为准。',
        '没有调大轮物理限制或缩小外形，没有修改生产src/devel或在线预算，没有接实车；隔离ROS节点已清理。',
        '本轮是固定输入后端实验，不是Gazebo、实车或在线MPPI交付性能证明。没有制作会误导为在线执行的GIF。']
    (out/'本轮实验简要结论.txt').write_text('\n'.join(lines)+'\n')
    plot(main_report,out/'约束残差对照.png')
    print(json.dumps(dict(out=str(out),evidence_valid=True,trial_count=len(checks)),ensure_ascii=False))


def plot(report,path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    phases=(1,2,3);x=np.arange(3)
    fig,axes=plt.subplots(1,2,figsize=(9.5,3.5),constrained_layout=True)
    for i,trial in enumerate(report['trials']):
        selected=[trial['phases'][j] for j in phases]
        ratios=[max(v['ratio_to_final_limit'] for v in p['last_canonical_wheel_audit']['native_10ms']['peaks'].values()) for p in selected]
        gaps=[p['iterate_audit']['candidates'][-1]['frozen_projected_20ms']['min_static_gap_m']*1000 for p in selected]
        for ax,values in zip(axes,(ratios,gaps)):
            ax.plot(x,values,'o-',markersize=4,label='D%d'%(i+1))
    axes[0].axhline(1,color='#777777',linestyle='--',linewidth=1)
    axes[1].axhline(0,color='#777777',linestyle='--',linewidth=1)
    axes[0].set_title('Worst wheel ratio (pass <= 1)')
    axes[1].set_title('Full projected gap (pass >= 0)')
    axes[0].set_ylabel('Peak / native hard limit')
    axes[1].set_ylabel('Millimetres; unchanged body and margin')
    for ax in axes:
        ax.set_xticks(x);ax.set_xticklabels(['Veto 0.6s','Veto 3s','Scaled 3s'],fontsize=9)
        ax.spines['top'].set_visible(False);ax.spines['right'].set_visible(False)
    axes[1].legend(fontsize=8,ncol=2)
    fig.suptitle('Final recorded accepted candidates - posterior diagnostics, not deliveries',fontsize=10)
    fig.savefig(path,dpi=150);plt.close(fig)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out',type=Path,required=True)
    main(p.parse_args().out.resolve())
