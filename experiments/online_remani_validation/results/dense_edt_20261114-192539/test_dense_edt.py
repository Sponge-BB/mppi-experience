import json,os,signal,socket,subprocess,time,xmlrpc.client
from pathlib import Path
x=json.loads(Path('/tmp/remani_dense_edt_version.json').read_text());out=Path(x['case']);port=11486
with socket.socket() as s:s.bind(('127.0.0.1',port))
env=os.environ.copy();env.pop('ROS_HOSTNAME',None);env.update(ROS_MASTER_URI='http://127.0.0.1:%d'%port,ROS_IP='127.0.0.1',ROS_LOG_DIR='/tmp/remani_dense_edt_test_logs',ROS_HOME='/tmp/remani_dense_edt_test_home',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MALLOC_ARENA_MAX='2',PYTHONNOUSERSITE='1',PYTHONDONTWRITEBYTECODE='1')
env['LD_LIBRARY_PATH']=x['stage_devel']+'/lib:'+x['build']+'/gtest/lib:/home/l/remani_ws/devel/lib:/opt/ros/noetic/lib'
master_log=(out/'test_master.log').open('w');master=subprocess.Popen(['/opt/ros/noetic/bin/roscore','-p',str(port)],env=env,stdout=master_log,stderr=subprocess.STDOUT,start_new_session=True);records=[]
try:
 for _ in range(100):
  if master.poll() is not None:raise RuntimeError('isolated master exited')
  try:
   if xmlrpc.client.ServerProxy(env['ROS_MASTER_URI']).getPid('/map_latency_test_ready')[0]==1:break
  except OSError:pass
  time.sleep(.1)
 else:raise RuntimeError('isolated master unavailable')
 for name in ['test_esdf_worker','test_grid_map_parallel','test_dynamic_point_buffer']:
  exe=Path(x['stage_devel'])/'lib/plan_env'/name;log=(out/(name+'.log')).open('w');start=time.monotonic();p=subprocess.Popen(['nice','-n','10',str(exe)],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);peak=0;minimum=1e30;reason=None
  while p.poll() is None:
   try:
    for line in Path('/proc/%d/status'%p.pid).read_text().splitlines():
     if line.startswith('VmHWM:'):peak=max(peak,int(line.split()[1])/1024)
   except OSError:pass
   avail=next(int(s.split()[1])/1024 for s in Path('/proc/meminfo').read_text().splitlines() if s.startswith('MemAvailable:'));minimum=min(minimum,avail)
   if avail<1500:reason='available_RAM_below_1500MiB'
   if peak>1200:reason='test_RSS_above_1200MiB'
   if time.monotonic()-start>180:reason='test_timeout_180s'
   if reason:
    os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5);break
   time.sleep(.05)
  log.close();r={'test':name,'exit_code':p.wait(),'elapsed_s':time.monotonic()-start,'peak_rss_mib':peak,'lowest_available_mib':minimum,'stop_reason':reason};records.append(r);print(json.dumps(r),flush=True)
  if r['exit_code']!=0 or reason:raise RuntimeError(name+' failed')
finally:
 os.killpg(master.pid,signal.SIGTERM)
 try:master.wait(timeout=8)
 except subprocess.TimeoutExpired:os.killpg(master.pid,signal.SIGKILL);master.wait()
 master_log.close();(out/'test_results.json').write_text(json.dumps({'isolated_master':env['ROS_MASTER_URI'],'hardware_connected':False,'records':records},indent=2)+'\n')
