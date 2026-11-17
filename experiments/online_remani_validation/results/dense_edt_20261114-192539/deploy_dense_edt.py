import hashlib, json, os, shutil, stat, subprocess
from pathlib import Path

x=json.loads(Path('/tmp/remani_dense_edt_version.json').read_text())
root=Path(x['root']); case=Path(x['case'])
source_hashes=json.loads((case/'build_source_sha256.json').read_text())
def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''): h.update(b)
    return h.hexdigest()
for rel,value in source_hashes.items():
    assert sha(root/rel)==value, 'Source changed during build: '+rel
for label in ('map_build',):
    result=json.loads((case/(label+'_result.json')).read_text())
    assert result['exit_code']==0 and result['stop_reason'] is None
rows=[]
for line in subprocess.check_output(['ps','-eo','pid,comm,args'],text=True).splitlines()[1:]:
    fields=line.strip().split(None,2)
    if len(fields)!=3 or fields[1] in ('bash','sh','timeout'): continue
    if any(word in fields[2] for word in ('roslaunch','/lib/remani_planner/remani_planner_node',
                                       'whole_body_mppi.py','piper_ctrl_single_node.py')):
        rows.append(line.strip())
assert not rows, 'Live robot processes; no deployment: '+repr(rows)
targets={'devel/lib/libplan_env.so':Path(x['stage_devel'])/'lib/libplan_env.so'}
before=json.loads((Path(x['baseline_dir'])/'files.json').read_text())
env=os.environ.copy()
env['LD_LIBRARY_PATH']=x['stage_devel']+'/lib:'+str(root/'devel/lib')+':/opt/ros/noetic/lib'
def inspect(path,label):
    dynamic=subprocess.check_output(['readelf','-d',str(path)],text=True)
    assert '/tmp/' not in dynamic, 'Temporary runtime dependency: '+str(path)
    result=subprocess.run(['ldd','-r',str(path)],env=env,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
    (case/(label+'_ldd.log')).write_text(result.stdout)
    (case/(label+'_dynamic.log')).write_text(dynamic)
    assert result.returncode==0 and 'not found' not in result.stdout and 'undefined symbol:' not in result.stdout
for i,(rel,stage) in enumerate(targets.items()):
    dest=root/rel
    assert sha(dest)==before[rel]['sha256'], 'Deployment no longer matches saved baseline: '+rel
    assert stat.S_IMODE(dest.stat().st_mode)==before[rel]['mode']
    inspect(stage,'staged_'+str(i))
assert all(r['exit_code']==0 and not r['stop_reason'] for r in json.loads((case/'test_results.json').read_text())['records'])
assert json.loads((case/'runs_final/summary.json').read_text())['native_uncapped_exact_match']
changed=[]
try:
    for rel,stage in targets.items():
        dest=root/rel; temp=dest.with_name(dest.name+'.dense-new')
        assert not temp.exists()
        shutil.copy2(stage,temp); os.chmod(temp,before[rel]['mode'])
        os.replace(temp,dest); changed.append(rel)
        assert sha(dest)==sha(stage)
    env['LD_LIBRARY_PATH']=str(root/'devel/lib')+':/opt/ros/noetic/lib'
    for i,rel in enumerate(targets): inspect(root/rel,'deployed_'+str(i))
    inspect(root/'devel/lib/remani_planner/remani_planner_node','current_planner')
except BaseException:
    for rel in changed:
        dest=root/rel; temp=dest.with_name(dest.name+'.dense-restore')
        shutil.copy2(Path(x['baseline_dir'])/'files'/rel,temp); os.replace(temp,dest)
    raise
result={'hardware_started':False,'host_process_check':rows,'source_sha256':source_hashes,
        'deployed':{rel:{'stage':str(stage),'sha256':sha(root/rel),
                         'stage_sha256':sha(stage),'mode':stat.S_IMODE((root/rel).stat().st_mode)}
                    for rel,stage in targets.items()},'link_check_passed':True}
(case/'deployment_result.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
