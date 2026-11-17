#!/usr/bin/python3
"""Deploy the two versioned tracker artifacts with ROS stopped; no robot commands."""
import hashlib,json,os,shutil,stat,subprocess
from pathlib import Path
x=json.loads(Path('/tmp/remani_tracker_reasons_version.json').read_text());root=Path(x['root']);case=Path(x['case']);baseline=Path(x['baseline_dir']);stage=Path(x['stage_devel'])
artifacts=['devel/lib/libdynamic_obstacle_tracker_core.so','devel/lib/mm_remani/dynamic_obstacle_tracker_node']
meta=json.loads((baseline/'files.json').read_text());sources=json.loads((case/'build_source_sha256.json').read_text())
assert set(sources)==set(x['files'])-set(artifacts) and len(sources)==4

def sha(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
 return h.hexdigest()

def source_check():
 for p,h in sources.items():assert sha(root/p)==h,'Source changed after build freeze: '+p

def stopped_check():
 pid1=Path('/proc/1/comm').read_text().strip();assert pid1 in ('systemd','init'),'Real host process view required'
 words=('roslaunch','/lib/remani_planner/remani_planner_node','whole_body_mppi.py','piper_ctrl_single_node.py','mm_controller_node','real_base_tracker.py','joint_cmd_bridge.py','/lib/mm_remani/dynamic_obstacle_tracker_node')
 active=[]
 for line in subprocess.check_output(['ps','-eo','pid,comm,args'],text=True).splitlines()[1:]:
  f=line.strip().split(None,2)
  if len(f)!=3 or f[1] in ('bash','sh','timeout'):continue
  if any(w in f[2] for w in words):active.append(line.strip())
 assert not active,'Live robot processes prevent deployment: '+repr(active)
 return {'pid1':pid1,'active':active}

def inspect(p,label,staged=False):
 dynamic=subprocess.check_output(['readelf','-d',str(p)],text=True);(case/(label+'_dynamic.log')).write_text(dynamic)
 assert '/tmp/' not in dynamic
 rpath=[l for l in dynamic.splitlines() if '(RUNPATH)' in l or '(RPATH)' in l]
 assert any(str(root/'devel/lib')+':/opt/ros/noetic/lib' in l for l in rpath), 'Unexpected RPATH'
 env=os.environ.copy();env['LD_LIBRARY_PATH']=(str(stage/'lib')+':' if staged else '')+str(root/'devel/lib')+':/opt/ros/noetic/lib'
 r=subprocess.run(['ldd','-r',str(p)],env=env,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT);(case/(label+'_ldd.log')).write_text(r.stdout)
 assert r.returncode==0 and 'not found' not in r.stdout and 'undefined symbol:' not in r.stdout,r.stdout
 if not staged:assert '/tmp/' not in r.stdout
 if p.name=='dynamic_obstacle_tracker_node':
  expected=(stage if staged else root/'devel')/'lib/libdynamic_obstacle_tracker_core.so'
  assert 'libdynamic_obstacle_tracker_core.so => '+str(expected) in r.stdout
 return rpath

source_check();build=json.loads((case/'tracker_build_result.json').read_text());assert build['exit_code']==0 and build['stop_reason'] is None and Path(build['build_dir'])==Path(x['build'])
processes=stopped_check();staged={p:stage/Path(p).relative_to('devel') for p in artifacts};hashes={}
for p in artifacts:
 d=root/p; saved=baseline/'files'/p;s=staged[p]
 assert d.is_file() and not d.is_symlink() and sha(d)==meta[p]['sha256']
 assert stat.S_IMODE(d.stat().st_mode)==meta[p]['mode']
 assert sha(saved)==meta[p]['sha256'] and s.is_file() and not s.is_symlink()
 hashes[p]=sha(s);inspect(s,'staged_'+s.name,True)
source_check();stopped_check();changed=[];temps=[]
try:
 for p in artifacts:
  d=root/p;t=d.with_name(d.name+'.tracker-diag-new');restore=d.with_name(d.name+'.tracker-diag-restore')
  assert not t.exists() and not t.is_symlink() and not restore.exists() and not restore.is_symlink()
  temps.append(t);shutil.copy2(staged[p],t);os.chmod(t,meta[p]['mode']);assert sha(t)==hashes[p] and sha(staged[p])==hashes[p]
 source_check();stopped_check()
 for p,t in zip(artifacts,temps):os.replace(t,root/p);changed.append(p)
 deployed={}
 for p in artifacts:
  assert sha(root/p)==hashes[p];rpath=inspect(root/p,'deployed_'+Path(p).name)
  deployed[p]={'sha256':sha(root/p),'stage_sha256':hashes[p],'before_sha256':meta[p]['sha256'],'mode':stat.S_IMODE((root/p).stat().st_mode),'rpath':rpath}
 source_check()
 result={'hardware_started':False,'host_process_check':processes,'source_sha256':sources,'deployed':deployed,'link_check_passed':True,'atomic_per_file_replace':True,'both_artifacts_replaced_while_ros_stopped':True}
 (case/'deployment_result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
except BaseException as error:
 for p in reversed(changed):
  d=root/p;r=d.with_name(d.name+'.tracker-diag-restore');shutil.copy2(baseline/'files'/p,r);os.chmod(r,meta[p]['mode']);os.replace(r,d);assert sha(d)==meta[p]['sha256']
 for t in temps:
  if t.exists():t.unlink()
 (case/'deployment_failure.json').write_text(json.dumps({'error':repr(error),'restored_artifacts':changed,'source_files_not_rolled_back':True,'do_not_start_until_resolved':True},indent=2)+'\n')
 raise
