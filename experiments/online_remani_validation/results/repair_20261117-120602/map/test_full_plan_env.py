#!/usr/bin/python3
import os,sys,json,time,signal,socket,resource,subprocess,xmlrpc.client,hashlib
from pathlib import Path
import xml.etree.ElementTree as ET
CASE=Path(__file__).resolve().parent
BUILD=CASE/'full_plan_env_build';DEVEL=CASE/'full_plan_env_devel'
LIB=DEVEL/'lib/libplan_env.so'
if not LIB.is_file():raise RuntimeError('Missing full staged library')
def limit():resource.setrlimit(resource.RLIMIT_AS,(1434*1024**2,1434*1024**2))
def available():return next(int(x.split()[1])*1024 for x in Path('/proc/meminfo').read_text().splitlines() if x.startswith('MemAvailable:'))
def group_rss(group):
 total=0
 for statfile in Path('/proc').glob('[0-9]*/stat'):
  try:
   f=statfile.read_text().rpartition(')')[2].split()
   if int(f[2])==group:total+=max(0,int(f[21]))*os.sysconf('SC_PAGE_SIZE')
  except (OSError,ValueError,IndexError):pass
 return total
with socket.socket() as s:
 s.bind(('127.0.0.1',0));port=s.getsockname()[1]
env=os.environ.copy();env.pop('ROS_HOSTNAME',None)
env.update(ROS_MASTER_URI='http://127.0.0.1:'+str(port),ROS_IP='127.0.0.1',ROS_LOG_DIR=str(CASE/'isolated_test_ros_logs'),ROS_HOME=str(CASE/'isolated_test_ros_home'),OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MALLOC_ARENA_MAX='2',PYTHONNOUSERSITE='1',PYTHONDONTWRITEBYTECODE='1')
env['LD_LIBRARY_PATH']=str(DEVEL/'lib')+':'+str(BUILD/'gtest/lib')+':/home/l/remani_ws/devel/lib:/opt/ros/noetic/lib'
master_log=(CASE/'full_tests_master.log').open('w')
master=subprocess.Popen(['/opt/ros/noetic/bin/roscore','-p',str(port)],env=env,stdout=master_log,stderr=subprocess.STDOUT,start_new_session=True,preexec_fn=limit)
records=[];ldd={};beg=time.monotonic()
try:
 for attempt in range(100):
  if master.poll() is not None:raise RuntimeError('Isolated ROS master exited')
  try:
   if xmlrpc.client.ServerProxy(env['ROS_MASTER_URI']).getPid('/staged_plan_env_tests')[0]==1:break
  except OSError:pass
  time.sleep(.1)
 else:raise RuntimeError('Isolated ROS master unavailable')
 for name in ['test_esdf_worker','test_grid_map_parallel','test_dynamic_point_buffer']:
  exe=DEVEL/'lib/plan_env'/name
  loaded=subprocess.check_output(['ldd',str(exe)],env=env,text=True)
  (CASE/(name+'.staged.ldd.txt')).write_text(loaded);ldd[name]=loaded
  matched=[line for line in loaded.splitlines() if 'libplan_env.so =>' in line]
  if len(matched)!=1 or str(LIB) not in matched[0]:raise RuntimeError('Test did not resolve staged plan_env library')
  if available()<1536*1024**2:raise RuntimeError('Insufficient memory before test')
  start=time.monotonic();peak=0;minimum=available();reason=None;xml=CASE/(name+'.staged.xml')
  with (CASE/(name+'.staged.log')).open('w') as log:
   p=subprocess.Popen(['nice','-n','10',str(exe),'--gtest_output=xml:'+str(xml)],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True,preexec_fn=limit)
   while p.poll() is None:
    peak=max(peak,group_rss(p.pid)+group_rss(master.pid));minimum=min(minimum,available())
    if minimum<1536*1024**2:reason='MemAvailable below 1.5 GiB'
    if peak>1229*1024**2:reason='Combined test/master group RSS above 1.2 GiB'
    if time.monotonic()-start>180:reason='180-second test timeout'
    if reason:
     os.killpg(p.pid,signal.SIGTERM)
     try:p.wait(timeout=5)
     except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
     break
    time.sleep(.1)
   rc=p.wait()
  parsed=ET.parse(xml).getroot().attrib if xml.exists() else {}
  row={'test':name,'exit_code':rc,'elapsed_s':time.monotonic()-start,'peak_test_and_master_rss_MiB':peak/1024**2,'minimum_MemAvailable_MiB':minimum/1024**2,'stop_reason':reason,'gtest_xml':parsed,'executable_sha256':hashlib.sha256(exe.read_bytes()).hexdigest()}
  records.append(row);print(json.dumps(row),flush=True)
  if rc or reason or parsed.get('failures')!='0' or parsed.get('errors')!='0':raise RuntimeError(name+' failed')
finally:
 if master.poll() is None:
  os.killpg(master.pid,signal.SIGTERM)
  try:master.wait(timeout=8)
  except subprocess.TimeoutExpired:os.killpg(master.pid,signal.SIGKILL);master.wait()
 master_log.close()
 (CASE/'full_tests_results.json').write_text(json.dumps({'isolated_master':env['ROS_MASTER_URI'],'hardware_connected':False,'elapsed_s':time.monotonic()-beg,'records':records,'staged_library_sha256':hashlib.sha256(LIB.read_bytes()).hexdigest(),'production_src_devel_untouched':True},indent=2)+'\n')
