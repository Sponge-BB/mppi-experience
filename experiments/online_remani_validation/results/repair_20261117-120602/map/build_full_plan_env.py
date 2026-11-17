#!/usr/bin/python3
import os,sys,json,time,signal,resource,subprocess,hashlib
from pathlib import Path
CASE=Path(__file__).resolve().parent
SOURCE=CASE/'full_plan_env_source';BUILD=CASE/'full_plan_env_build';DEVEL=CASE/'full_plan_env_devel'
EXPECTED=json.loads((CASE/'full_plan_env_source_manifest.json').read_text())['staged_source']
def verify():
 for name,meta in EXPECTED.items():
  f=SOURCE/name
  if 'sha256' in meta and hashlib.sha256(f.read_bytes()).hexdigest()!=meta['sha256']:raise RuntimeError('Source changed: '+name)
  if 'symlink' in meta and os.readlink(str(f))!=meta['symlink']:raise RuntimeError('Link changed')
def limits():resource.setrlimit(resource.RLIMIT_AS,(1434*1024**2,1434*1024**2))
def available():return next(int(x.split()[1])*1024 for x in Path('/proc/meminfo').read_text().splitlines() if x.startswith('MemAvailable:'))
def monitored(label,cmd):
 verify()
 if available()<1536*1024**2:raise RuntimeError('MemAvailable below 1.5 GiB before launch')
 env=os.environ.copy();env.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MALLOC_ARENA_MAX='2',PYTHONNOUSERSITE='1',PYTHONDONTWRITEBYTECODE='1')
 start=time.monotonic();peak=0;minimum=available();reason=None
 with (CASE/(label+'.log')).open('w') as log:
  p=subprocess.Popen(cmd,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,preexec_fn=limits)
  while p.poll() is None:
   rss=0
   for statfile in Path('/proc').glob('[0-9]*/stat'):
    try:
     fields=statfile.read_text().rpartition(')')[2].split()
     if int(fields[2])==p.pid:rss+=max(0,int(fields[21]))*os.sysconf('SC_PAGE_SIZE')
    except (OSError,ValueError,IndexError):pass
   peak=max(peak,rss);minimum=min(minimum,available())
   if minimum<1536*1024**2:reason='MemAvailable below 1.5 GiB'
   if rss>1229*1024**2:reason='Build process group RSS above 1.2 GiB'
   if time.monotonic()-start>900:reason='900-second timeout'
   if reason:
    os.killpg(p.pid,signal.SIGTERM)
    try:p.wait(timeout=5)
    except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
    break
   time.sleep(.25)
  rc=p.wait()
 result={'command':cmd,'exit_code':rc,'elapsed_s':time.monotonic()-start,'peak_process_group_rss_MiB':peak/1024**2,'minimum_MemAvailable_MiB':minimum/1024**2,'stop_reason':reason,'RLIMIT_AS_MiB':1434,'max_build_jobs':1,'production_src_devel_untouched':True}
 (CASE/(label+'.json')).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result),flush=True)
 verify()
 if rc or reason:raise RuntimeError(label+' failed')
if BUILD.exists() or DEVEL.exists():raise RuntimeError('Refusing nonempty/previous build or devel directory')
monitored('full_configure',['nice','-n','15','cmake','-S',str(SOURCE),'-B',str(BUILD),'-DCATKIN_DEVEL_PREFIX='+str(DEVEL),'-DCMAKE_PREFIX_PATH=/home/l/remani_ws/devel;/opt/ros/noetic','-DPYTHON_EXECUTABLE=/usr/bin/python3','-DCATKIN_ENABLE_TESTING=ON','-DCMAKE_BUILD_WITH_INSTALL_RPATH=ON','-DCMAKE_INSTALL_RPATH=/home/l/remani_ws/devel/lib;/opt/ros/noetic/lib','-DCMAKE_CXX_COMPILER_LAUNCHER='+str(CASE/'full_compile_launcher.sh'),'-DCMAKE_EXPORT_COMPILE_COMMANDS=ON'])
monitored('full_build',['nice','-n','15','cmake','--build',str(BUILD),'--parallel','1','--target','plan_env','test_esdf_worker','test_grid_map_parallel','test_dynamic_point_buffer'])
