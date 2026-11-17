import json,os,signal,subprocess,time,sys
from pathlib import Path
x=json.loads(Path('/tmp/remani_live_witness_version.json').read_text());case=Path(x['case']);build=Path(x['build']);env=os.environ.copy();env.update(PYTHONNOUSERSITE='1',PYTHONDONTWRITEBYTECODE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MALLOC_ARENA_MAX='2')
records=[r for r in json.loads((case/'build_results.json').read_text()) if r['exit_code']==0] if len(sys.argv)>1 else []
def run(cmd,label):
 log=(case/(label+'.log')).open('w');p=subprocess.Popen(['nice','-n','15']+cmd,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);beg=time.monotonic();peak=0;min_avail=1e30;reason=None
 while p.poll() is None:
  rss=0
  for q in Path('/proc').glob('[0-9]*/stat'):
   try:
    f=q.read_text().rpartition(')')[2].split()
    if int(f[2])==p.pid:rss+=max(0,int(f[21]))*os.sysconf('SC_PAGE_SIZE')
   except (OSError,ValueError,IndexError):pass
  avail=next(int(s.split()[1])*1024 for s in Path('/proc/meminfo').read_text().splitlines() if s.startswith('MemAvailable:'));peak=max(peak,rss);min_avail=min(min_avail,avail)
  if avail<1500*1024**2:reason='available_RAM_below_1500MiB'
  if rss>1800*1024**2:reason='own_build_RSS_above_1800MiB'
  if time.monotonic()-beg>900:reason='own_build_time_limit_900s'
  if reason:
   os.killpg(p.pid,signal.SIGTERM)
   try:p.wait(timeout=5)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
   break
  time.sleep(.5)
 log.close();r={'command':cmd,'exit_code':p.wait(),'elapsed_s':time.monotonic()-beg,'peak_build_group_rss_mib':peak/1024**2,'lowest_available_mib':min_avail/1024**2,'stop_reason':reason,'jobs':1,'clean_build':True,'runtime_destination_untouched':True};records.append(r);(case/'build_results.json').write_text(json.dumps(records,indent=2)+'\n');print(json.dumps(dict(label=label,**r)),flush=True)
 if r['exit_code'] or reason:raise SystemExit(1)
for pkg,targets in [('plan_env',['plan_env','test_esdf_worker','test_grid_map_parallel','test_dynamic_point_buffer']),('mm_config',['mm_config']),('traj_opt',['traj_opt']),('plan_manage',['remani_planner_node'])]:
 if len(sys.argv)>1 and pkg=='plan_env':continue
 dest=build/pkg
 run(['cmake','-S',str(Path(x['root'])/'src/mm_remani_planner'/pkg),'-B',str(dest),'-DCATKIN_DEVEL_PREFIX='+x['stage_devel'],'-DCMAKE_PREFIX_PATH='+x['stage_devel']+';'+x['root']+'/devel;/opt/ros/noetic','-DCMAKE_CXX_FLAGS=-I/usr/include/eigen3 -I/usr/include/opencv4 -I/usr/include/pcl-1.10','-DPYTHON_EXECUTABLE=/usr/bin/python3','-DCATKIN_ENABLE_TESTING=ON','-DCMAKE_BUILD_WITH_INSTALL_RPATH=ON','-DCMAKE_INSTALL_RPATH='+x['root']+'/devel/lib;/opt/ros/noetic/lib','-DCMAKE_CXX_COMPILER_LAUNCHER=/tmp/remani_dense_edt_no_debug_compile.sh'],pkg+'_configure')
 run(['cmake','--build',str(dest),'--parallel','1','--target']+targets,pkg+'_build')
