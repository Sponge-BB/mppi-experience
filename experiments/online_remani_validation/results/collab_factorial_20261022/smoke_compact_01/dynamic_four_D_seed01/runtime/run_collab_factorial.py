#!/usr/bin/env python3
"""Isolated native ROS target x MPPI-prefix experiment, with paired diagnostics."""
import argparse
import collections
import gzip
import hashlib
import json
import os
from pathlib import Path
import random
import re
import shutil
import socket
import subprocess
import time
import xml.etree.ElementTree as ET

from prepare import HERE, PORT, prepare
from run_dual_timescale import verify_production, stop_owned, audit_graph
from run_resource_comparison import Monitor
from report_dual_timescale import summarize_trial
from audit_dual_timescale import audit_trial, build_integrity

GROUPS={'A':(False,False),'B':(False,True),'C':(True,False),'D':(True,True)}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def patch_runtime(out, random_seed):
    runtime=out/'runtime'
    path=runtime/'local_execution.py';text=path.read_text()
    anchor='            node.execution_pub.publish(path)'
    assert text.count(anchor)==1
    text=text.replace(anchor,anchor+'''
        # Telemetry follows command publication. It does not authorize motion.
        from collab_seed_export import publish_seed
        publish_seed(node,checked,validated_count,published,generation,revision,odom_stamp,cloud_stamp,stamp)
''')
    path.write_text(text)
    path=runtime/'mppi_node.py';text=path.read_text()
    anchor='self.handle=self.lib.online_create(ptr(p),ptr(body),len(body),1)'
    assert text.count(anchor)==1
    text=text.replace(anchor,'self.handle=self.lib.online_create(ptr(p),ptr(body),len(body),int(rospy.get_param("~random_seed",1)))')
    text=text.replace('        body=np.ascontiguousarray(', '        self.limits_margin=limits["margin"]\n        body=np.ascontiguousarray(',1)
    path.write_text(text)
    path=runtime/'search_recording.py';text=path.read_text()
    anchor="                np.savez(self.out / ('%06d.npz' % self.stats['written']), **item)"
    assert text.count(anchor)==1
    text=text.replace(anchor,'''                rows=item['rows']
                if len(rows):
                    selected=np.unique(rows[rows[:,7]>.5,0]).tolist()
                    ids=selected+[v for v in np.unique(rows[:,0]) if v not in selected]
                    item['rows']=rows[np.isin(rows[:,0],ids[:24])]
                item['export_subset_limit']=24
'''+anchor)
    path.write_text(text)
    manifest=json.loads((out/'manifest.json').read_text())
    for name in ('local_execution.py','mppi_node.py','search_recording.py'):
        manifest['experiment_sources_sha256'][name]=sha(runtime/name)
    manifest['factorial_runtime_patch']=dict(random_seed=random_seed,
        feedback='post-command checked state prediction; initialization only, not committed motion',
        search='at most 24 real candidate traces per stored frame; no fabricated samples')
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2,ensure_ascii=False)+'\n')


def prepare_trial(out,build,scene,group,random_seed,duration,paired=False,record_search=False):
    prepare(out,build,'mppi',scene,duration,projection_progress=True,scenario_start='motion',
            perception='classified',execution='local',record_search=record_search,prediction_horizon=3.)
    patch_runtime(out,random_seed)
    tree=ET.parse(out/'experiment.launch');launch=tree.getroot()
    for node in launch.findall('node'):
        if node.get('name')=='base_mppi':
            ET.SubElement(node,'param',name='random_seed',value=str(random_seed),type='int')
    package=out/'ros_pkgs/online_remani_experiment'
    (package/'dual_timescale_observer.py').symlink_to(out/'runtime/dual_timescale_observer.py')
    ET.SubElement(launch,'node',pkg='online_remani_experiment',type='dual_timescale_observer.py',
                  name='dual_timescale_observer',output='log',required='true',args='--out '+str(out))
    tree.write(out/'experiment.launch',encoding='unicode')
    manifest=json.loads((out/'manifest.json').read_text())
    rolling,prefix=GROUPS[group]
    manifest['factorial']=dict(group=group,rolling=rolling,prefix=prefix,seed=random_seed,
        paired_diagnostic=paired,optimizer_budget_unchanged=True,retry_unchanged=True,
        final_goal_unchanged=True,collision_implementation_unchanged=True)
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2,ensure_ascii=False)+'\n')


def compress_new_records(out):
    """Lossless only; old experiment files are never touched."""
    archive={}
    names=['execution_rollouts.jsonl','observer_events.jsonl','cpu_samples.jsonl',
           'mppi_diagnostics.json','events.json','trajectories.json']
    for name in names:
        path=out/name
        if not path.exists():continue
        target=path.with_name(name+'.gz');digest=sha(path)
        with path.open('rb') as source,gzip.GzipFile(filename=str(target),mode='wb',mtime=0) as dest:
            shutil.copyfileobj(source,dest)
        with gzip.open(target,'rb') as source:
            check=hashlib.sha256()
            for chunk in iter(lambda:source.read(1024*1024),b''):check.update(chunk)
        if check.hexdigest()!=digest:raise RuntimeError('lossless archive verification failed: '+name)
        archive[name]=dict(original_sha256=digest,original_bytes=path.stat().st_size,
                           gzip_sha256=sha(target),gzip_bytes=target.stat().st_size)
        path.unlink()  # recoverable byte-for-byte from the checked gzip archive
    (out/'compression_audit.json').write_text(json.dumps(archive,indent=2)+'\n')


def trial(out,build,scene,group,random_seed,duration=48.,paired=False,record_search=False):
    with socket.socket() as sock:
        if sock.connect_ex(('127.0.0.1',PORT))==0:raise RuntimeError('isolated master port already in use')
    before=verify_production(build)
    if shutil.disk_usage(out.parent).free<250*1024**2:raise RuntimeError('less than 250MiB free; refusing to exhaust disk')
    prepare_trial(out,build,scene,group,random_seed,duration,paired,record_search)
    env={k:v for k,v in os.environ.items() if not re.search('TOKEN|SECRET|PASSWORD|CREDENTIAL|AUTH|API_KEY',k,re.I)
         and not k.startswith(('FAILURE_','RESOURCE_','COLLAB_'))}
    rolling,prefix=GROUPS[group]
    env.update(ROS_MASTER_URI='http://127.0.0.1:%d'%PORT,ROS_IP='127.0.0.1',ROS_HOSTNAME='localhost',
        ROS_HOME=str(out/'ros_home'),ROS_LOG_DIR=str(out/'roslogs'),
        ROS_PACKAGE_PATH=str(out/'ros_pkgs')+':'+env.get('ROS_PACKAGE_PATH',''),
        DUAL_TIMELINE_PATH=str(out/'dual_timeline.jsonl'),RESOURCE_TRACE_PATH=str(out/'solver_trace.jsonl'),
        FAILURE_TRACE_PATH=str(out/'failure_trace.jsonl'),FAILURE_PROFILE_DISABLED='1',
        COLLAB_ROLLING=str(int(rolling)),COLLAB_PREFIX=str(int(prefix)),COLLAB_SEED=str(random_seed),
        COLLAB_PAIR=str(int(paired)),COLLAB_PAIR_DONE=str(out/'paired_done.json'),
        FAILURE_OWNED_PID_FILE=str(out/'owned_pids.txt'))
    started=time.monotonic();audited=False;failure=None;paired_finished=False
    with (out/'launch_stdout.log').open('w') as log:
        process=subprocess.Popen(['roslaunch','-p',str(PORT),str(out/'experiment.launch')],env=env,
            stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        monitor=Monitor(process.pid,out);monitor.start()
        try:
            while process.poll() is None and not (out/'result.json').exists():
                elapsed=time.monotonic()-started
                owned=monitor.registered
                if '/algorithm_simulator' in owned and '/base_mppi' in owned:
                    pid_file=out/'owned_pids.txt'
                    if not pid_file.exists():pid_file.write_text('%d %d\n'%(owned['/algorithm_simulator'],owned['/base_mppi']))
                if not audited and elapsed>13:audit_graph(out);audited=True
                if paired and (out/'paired_done.json').exists():paired_finished=True;time.sleep(.4);break
                if elapsed>duration+75:raise RuntimeError('trial wall deadline')
                time.sleep(.2)
            if (out/'result.json').exists():time.sleep(.4)
        except Exception as exc:failure='%s: %s'%(type(exc).__name__,exc)
        finally:
            stop_owned(process);monitor.close()
    logs=(out/'launch_stdout.log').read_text(errors='replace')
    status=dict(returncode=process.returncode,result_written=(out/'result.json').exists(),
        required_process_failed=bool(re.search(r'REQUIRED process \[[^\]]+\] has died',logs)),
        graph_audited=audited,monitored_node_pids=monitor.registered,runner_error=failure,
        paired_diagnostic=paired,paired_finished=paired_finished,elapsed_wall_s=time.monotonic()-started,
        production_hashes_unchanged=before==verify_production(build))
    (out/'run_status.json').write_text(json.dumps(status,indent=2)+'\n')
    if failure or status['required_process_failed'] or not audited:raise RuntimeError('invalid trial retained: '+str(out))
    if paired:
        events=[json.loads(v) for v in (out/'dual_timeline.jsonl').read_text().splitlines() if v.strip()]
        results=[v for v in events if v['event']=='collab_pair_result']
        valid=paired_finished and len(results)==8 and all(v['stable'] for v in results)
        summary=dict(paired_integrity=valid,results=results,
                     scope='same frozen task/map/time; optimizer acceptance, not live delivery; paused run excluded from online performance')
        (out/'paired_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
        if not valid:raise RuntimeError('paired invariants failed, retained: '+str(out))
    else:
        if not status['result_written']:raise RuntimeError('missing numerical plant result')
        report=summarize_trial(out,out/'analysis')
        integrity=build_integrity(build,{})
        audit=audit_trial(out,integrity,out/'analysis/summary.json')
        (out/'trial_audit.json').write_text(json.dumps(audit,indent=2,ensure_ascii=False)+'\n')
        if not audit['valid_experiment']:raise RuntimeError('audit failure retained: '+str(out)+' '+str(audit['errors']))
    print('COMPLETE',out.name,'paired' if paired else json.loads((out/'result.json').read_text())['reason'],flush=True)
    compress_new_records(out)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--build',type=Path,default=HERE/'builds/collab_factorial_20261022_03')
    parser.add_argument('--groups',nargs='+',choices=list(GROUPS),default=list(GROUPS))
    parser.add_argument('--scenes',nargs='+',default=['empty','static','dynamic_head_on_center','dynamic_four'])
    parser.add_argument('--seeds',nargs='+',type=int,default=[1])
    parser.add_argument('--duration',type=float,default=48.)
    parser.add_argument('--paired',action='store_true')
    parser.add_argument('--search',action='store_true')
    args=parser.parse_args();out=args.out.resolve();build=args.build.resolve()
    out.mkdir(parents=True,exist_ok=False)
    plan=dict(build=str(build),groups=args.groups,scenes=args.scenes,seeds=args.seeds,
              paired=args.paired,duration=args.duration,search=args.search,date='2026-10-22 Asia/Shanghai',
              note='online matched settings/seeds, not identical closed-loop inputs; frozen paired solves separate')
    (out/'experiment_plan.json').write_text(json.dumps(plan,indent=2)+'\n')
    jobs=[(scene,group,seed) for seed in args.seeds for scene in args.scenes for group in args.groups]
    # Counterbalance group order without changing scenario or controller seed.
    random.Random(20261022).shuffle(jobs)
    for scene,group,seed in jobs:
        trial(out/('%s_%s_seed%02d'%(scene,group,seed)),build,scene,group,seed,args.duration,args.paired,args.search)
    (out/'batch_complete.json').write_text(json.dumps(dict(complete=True,trials=len(jobs)))+'\n')


if __name__=='__main__':main()
