#!/usr/bin/python3
"""Deploy only the checked planner binary; requires the real host process view."""
import hashlib, json, os, shutil, stat, subprocess
from pathlib import Path

x=json.loads(Path('/tmp/remani_fast_refresh_build.json').read_text())
root=Path(x['root']);case=Path(x['case'])
relative='devel/lib/remani_planner/remani_planner_node'
destination=root/relative
staged=Path(x['stage_devel'])/'lib/remani_planner/remani_planner_node'
baseline=Path(x['baseline_dir'])
before=json.loads((baseline/'files.json').read_text())[relative]
source_hashes=json.loads((case/'build_source_sha256.json').read_text())
required_sources=set(x['files'])-{relative}
assert len(required_sources)==7,'Expected all seven stage2 source/script/header/test files'
assert required_sources.issubset(source_hashes),'Missing build source freeze'

def sha(path):
 h=hashlib.sha256()
 with path.open('rb') as stream:
  for block in iter(lambda:stream.read(1024*1024),b''):h.update(block)
 return h.hexdigest()

def source_check():
 for path,expected in source_hashes.items():
  assert sha(root/path)==expected,'Source changed after build freeze: '+path

def robot_process_check():
 pid1=Path('/proc/1/comm').read_text().strip()
 assert pid1 in ('systemd','init'),'Masked process namespace: run deployment with the real host process view'
 active=[]
 words=('roslaunch','/lib/remani_planner/remani_planner_node','whole_body_mppi.py',
        'piper_ctrl_single_node.py','mm_controller_node','real_base_tracker.py','joint_cmd_bridge.py')
 for line in subprocess.check_output(['ps','-eo','pid,comm,args'],text=True).splitlines()[1:]:
  fields=line.strip().split(None,2)
  if len(fields)!=3 or fields[1] in ('bash','sh','timeout'):continue
  if any(word in fields[2] for word in words):active.append(line.strip())
 assert not active,'Live robot processes; deployment prohibited: '+repr(active)
 return {'pid1':pid1,'active':active}

source_check()
build=json.loads((case/'planner_build_result.json').read_text())
assert build['exit_code']==0 and build['stop_reason'] is None and build['clean_build'] is True
assert Path(build['build_dir'])==Path(x['build'])
processes=robot_process_check()
assert destination.is_file() and not destination.is_symlink()
assert sha(destination)==before['sha256'],'Runtime binary differs from saved baseline'
assert stat.S_IMODE(destination.stat().st_mode)==before['mode']
saved=baseline/'files'/relative
assert sha(saved)==before['sha256'] and stat.S_IMODE(saved.stat().st_mode)==before['mode']
assert staged.is_file() and not staged.is_symlink()
staged_hash=sha(staged)
env=os.environ.copy();env['LD_LIBRARY_PATH']=str(root/'devel/lib')+':/opt/ros/noetic/lib'

def inspect(path,label):
 dynamic=subprocess.check_output(['readelf','-d',str(path)],text=True)
 (case/(label+'_dynamic.log')).write_text(dynamic)
 assert '/tmp/' not in dynamic,'Temporary runtime dependency'
 runtime=[line for line in dynamic.splitlines() if '(RPATH)' in line or '(RUNPATH)' in line]
 assert any(str(root/'devel/lib')+':/opt/ros/noetic/lib' in line for line in runtime),'Unexpected runtime search path'
 result=subprocess.run(['ldd','-r',str(path)],env=env,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
 (case/(label+'_ldd.log')).write_text(result.stdout)
 assert result.returncode==0 and 'not found' not in result.stdout and 'undefined symbol:' not in result.stdout
 assert '/tmp/' not in result.stdout,'Runtime linker resolved temporary dependency'
 assert 'libplan_env.so => '+str(root/'devel/lib/libplan_env.so') in result.stdout
 return runtime

inspect(staged,'staged_planner')
source_check();robot_process_check()
temp=destination.with_name(destination.name+'.fast-refresh-new')
restore=destination.with_name(destination.name+'.fast-refresh-restore')
assert not temp.exists() and not temp.is_symlink() and not restore.exists() and not restore.is_symlink()
changed=False
try:
 shutil.copy2(staged,temp);os.chmod(temp,before['mode'])
 assert sha(temp)==staged_hash and sha(staged)==staged_hash,'Build artifact changed during deployment'
 source_check()
 os.replace(temp,destination);changed=True
 assert sha(destination)==staged_hash
 rpath=inspect(destination,'deployed_planner')
 source_check()
 result={'hardware_started':False,'host_process_check':processes,'source_sha256':source_hashes,
         'deployed':{relative:{'stage':str(staged),'sha256':sha(destination),'stage_sha256':staged_hash,
                              'before_sha256':before['sha256'],'mode':stat.S_IMODE(destination.stat().st_mode)}},
         'link_check_passed':True,'rpath':rpath,'atomic_replace':True,'only_planner_binary_replaced':True}
 (case/'deployment_result.json').write_text(json.dumps(result,indent=2)+'\n')
except BaseException as error:
 if changed:
  shutil.copy2(saved,restore);os.chmod(restore,before['mode']);os.replace(restore,destination)
  assert sha(destination)==before['sha256']
 if temp.exists():temp.unlink()
 (case/'deployment_failure.json').write_text(json.dumps({'error':repr(error),'previous_binary_restored':changed,
  'source_files_not_rolled_back':True,'do_not_start_until_resolved':True},indent=2)+'\n')
 raise
print(json.dumps(result,indent=2))
