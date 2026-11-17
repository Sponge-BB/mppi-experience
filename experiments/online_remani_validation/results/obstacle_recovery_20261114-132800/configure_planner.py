#!/usr/bin/python3
"""Prepare a new isolated planner build only; never deploy or start ROS."""
import json, os, subprocess
from pathlib import Path

state=Path('/tmp/remani_obstacle_recovery_build.json')
x=json.loads(state.read_text());case=Path(x['case'])
for key in ('build','stage_devel'):
 p=Path(x[key])
 if p.exists():raise RuntimeError('clean preparation requires absent directory: '+str(p))
command=['nice','-n','15','cmake','-S',x['root']+'/src/mm_remani_planner/plan_manage','-B',x['build'],
 '-DCATKIN_DEVEL_PREFIX='+x['stage_devel'],
 '-DCMAKE_PREFIX_PATH='+x['root']+'/devel;/opt/ros/noetic',
 '-DPYTHON_EXECUTABLE=/usr/bin/python3','-DCATKIN_ENABLE_TESTING=OFF',
 '-DCMAKE_BUILD_WITH_INSTALL_RPATH=ON',
 '-DCMAKE_INSTALL_RPATH='+x['root']+'/devel/lib;/opt/ros/noetic/lib',
 '-DCMAKE_CXX_COMPILER_LAUNCHER=/tmp/remani_obstacle_recovery_no_debug_compile.sh']
env=os.environ.copy();env.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MALLOC_ARENA_MAX='2')
with (case/'configure_planner.log').open('w') as log:
 result=subprocess.run(command,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=120)
record={'command':command,'returncode':result.returncode,'compiler_append':'-g0','target_build_started':False,'runtime_destination_untouched':True}
if result.returncode==0:
 build=Path(x['build']);flags=(build/'CMakeFiles/remani_planner_node.dir/flags.make').read_text();link=(build/'CMakeFiles/remani_planner_node.dir/link.txt').read_text()
 checks={'no_elevated_rosconsole_minimum':'ROSCONSOLE_MIN_SEVERITY' not in flags,
  'runtime_rpath':'-Wl,-rpath,'+x['root']+'/devel/lib:/opt/ros/noetic/lib ' in link,
  'current_plan_env':x['root']+'/devel/lib/libplan_env.so' in link,
  'current_dependencies':all(x['root']+'/devel/lib/lib'+name+'.so' in link for name in ('path_searching','traj_opt','mm_config','traj_utils')),
  'g0_compiler_launcher':'/tmp/remani_obstacle_recovery_no_debug_compile.sh' in (build/'CMakeFiles/remani_planner_node.dir/build.make').read_text()}
 record['checks']=checks
 (case/'planner_compile_flags.txt').write_text(flags)
 (case/'planner_link_command.txt').write_text(link)
 if not all(checks.values()):record['returncode']=1
(case/'configure_planner.json').write_text(json.dumps(record,indent=2)+'\n')
print(json.dumps(record,indent=2))
raise SystemExit(record['returncode'])
