#!/usr/bin/python3
"""One-session, read-only capture using native rosbag; no motion publication."""
import datetime,hashlib,json,os,signal,socket,subprocess,time,xmlrpc.client
from pathlib import Path
context=json.loads(Path('/tmp/remani_acceptance_current.json').read_text());out=Path(context['output']);root=Path('/home/l/remani_ws');MIB=1024**2
old=json.loads((root/'experiments/online_remani_validation/results/real_dynamic_20261116-092652/recording_start.json').read_text());core=list(old['topics'])
for topic in ['/gazebo/joint_states','/arm_status','/piper_motor_enable_states','/dynamic_obstacle_tracker/stationary_background_anchor_count','/clock']:
 if topic not in core:core.append(topic)
geometry=['/velodyne_points','/velodyne_points_mapping','/livox/lidar_no_ground','/cloud_registered','/map_generator/global_cloud']
env=os.environ.copy();env.update(ROS_MASTER_URI=context['master'],ROS_IP=context['local_ip'],PYTHONNOUSERSITE='1',OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MALLOC_ARENA_MAX='2');env.pop('ROS_HOSTNAME',None)
children=[];running=True;beg=time.monotonic();reason=None;original_geometry_size=0
socket.setdefaulttimeout(1.5)
def stop(*_):
 global running
 running=False
for signum in (signal.SIGINT,signal.SIGTERM):signal.signal(signum,stop)
def write(name,data):
 p=out/name;t=p.with_name(p.name+'.tmp');t.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n');t.replace(p)
def spawn(tag,cmd):
 log=(out/(tag+'_recorder.log')).open('xb');p=subprocess.Popen(cmd,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
 children.append({'tag':tag,'pid':p.pid,'command':cmd,'process':p,'log_handle':log})
spawn('core',['/opt/ros/noetic/lib/rosbag/record','--lz4','--buffsize=16','--chunksize=256','--duration=600','--size=256','--min-space=2G','--tcpnodelay','-O',str(out/'core.bag'),*core])
spawn('geometry',['/opt/ros/noetic/lib/rosbag/record','--lz4','--buffsize=32','--chunksize=256','--duration=600','--size=1536','--min-space=2G','--tcpnodelay','-O',str(out/'geometry.bag'),*geometry])
spawn('can',['/usr/bin/python3',str(out/'readonly_can_observer.py')])
spawn('map_observer',['/usr/bin/python3',str(out/'map_observer.py')])
serial=[{k:v for k,v in q.items() if k not in ('process','log_handle')} for q in children]
write('recording_start.json',dict(context,recorders=serial,core_topics=core,geometry_topics=geometry,geometry_sampling='Full published-message rate; no intentional thinning',limits={'duration_s':600,'core_mib':256,'geometry_mib':1536,'can_mib':64,'map_events_mib':256},caveat='Independent TCPROS subscriptions can drop before recording; monitor buffer warnings, publisher sequence/source stamps, consumer output source stamps and CAN socket drops. Kernel CAN time is not motor transmit time.'))
write('phase_marks.json',{'marks':[{'phase':'recording_prelaunch','wall':time.time(),'local':datetime.datetime.now().isoformat()}]})
master=xmlrpc.client.ServerProxy(context['master']);last_graph=0;last_capture=0;startup_captured=False;robot_live_seen=False;resources=(out/'resources.jsonl').open('a',buffering=1)
def snapshot_runtime(label,state):
 write(label+'_graph.json',{'wall':time.time(),'state':state})
 try:write(label+'_parameters.json',{'wall':time.time(),'parameters':master.getParam('/repair_capture','/')[2]})
 except Exception as e:write(label+'_parameters_error.json',{'error':repr(e)})
 proc=[]
 for p in Path('/proc').glob('[0-9]*/cmdline'):
  try:
   cmd=p.read_bytes().replace(b'\0',b' ').decode(errors='replace')
   if any(w in cmd for w in ['/lib/remani_planner/remani_planner_node','whole_body_mppi.py','piper_ctrl_single_node.py','dynamic_obstacle_tracker_node','velodyne_transformer.py']):
    pid=int(p.parent.name);proc.append({'pid':pid,'cmdline':cmd,'exe':os.readlink(p.parent/'exe'),'stat':(p.parent/'stat').read_text(),'maps':(p.parent/'maps').read_text()})
  except (OSError,ValueError):pass
 write(label+'_processes.json',{'wall':time.time(),'processes':proc})
try:
 while running and time.monotonic()-beg<600:
  now=time.monotonic();mem={l.split(':',1)[0]:int(l.split()[1]) for l in Path('/proc/meminfo').read_text().splitlines() if l.startswith(('MemAvailable:','SwapFree:'))};alerts=[]
  sizes={p.name:p.stat().st_size for p in out.glob('*.bag*') if p.is_file()};rec=[]
  if sum(v for k,v in sizes.items() if k.startswith('geometry'))>1228*MIB:alerts.append('geometry_budget_above_80_percent')
  for q in children:
   p=q['process'];row={'tag':q['tag'],'pid':q['pid'],'exit_code':p.poll()}
   try:
    v=Path('/proc')/str(p.pid);row['status']={l.split(':',1)[0]:l.split(':',1)[1].strip() for l in (v/'status').read_text().splitlines() if l.startswith(('VmRSS:','VmHWM:','Threads:'))};row['stat']=(v/'stat').read_text();row['io']=(v/'io').read_text()
   except OSError:pass
   rec.append(row)
   if p.poll() is not None:alerts.append(q['tag']+'_recorder_exited_'+str(p.returncode))
   try:
    with (out/(q['tag']+'_recorder.log')).open('rb') as f:
     f.seek(max(0,f.seek(0,2)-8192));tail=f.read().decode(errors='replace')
    if 'buffer exceeded' in tail.lower() or 'dropping' in tail.lower():alerts.append(q['tag']+'_buffer_warning')
   except OSError:pass
  syscan={}
  for key in ['rx_packets','rx_dropped','rx_errors','tx_packets','tx_dropped','tx_errors']:
   p=Path('/sys/class/net/can1/statistics')/key
   if p.exists():syscan[key]=int(p.read_text())
  active=[]
  for p in Path('/proc').glob('[0-9]*/stat'):
   try:
    cmd=(p.parent/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
    if any(w in cmd for w in ['/lib/remani_planner/remani_planner_node','whole_body_mppi.py','piper_ctrl_single_node.py','dynamic_obstacle_tracker_node','velodyne_transformer.py']):
     a={'pid':int(p.parent.name),'cmd':cmd,'stat':p.read_text(),'schedstat':(p.parent/'schedstat').read_text()}
     if 'piper_ctrl_single_node.py' in cmd:
      a['threads']=[{'stat':t.read_text(),'schedstat':(t.parent/'schedstat').read_text()} for t in (p.parent/'task').glob('*/stat')]
     active.append(a)
   except (OSError,ValueError):pass
  resources.write(json.dumps({'wall':time.time(),'mono':now,'mem_kib':mem,'recorders':rec,'can1_counters':syscan,'robot_processes':active})+'\n')
  if now-last_graph>3:
   last_graph=now
   try:
    response=master.getSystemState('/repair_capture');state=response[2];pubs={t:n for t,n in state[0]};subs={t:n for t,n in state[1]};coverage={t:{'publishers':pubs.get(t,[]),'recorders':[n for n in subs.get(t,[]) if '/record_' in n]} for t in core+geometry}
    buses={}
    for node in sorted({n for t in core+geometry for n in subs.get(t,[]) if '/record_' in n}):
     try:uri=master.lookupNode('/repair_capture',node)[2];buses[node]=xmlrpc.client.ServerProxy(uri).getBusStats('/repair_capture')
     except Exception as e:buses[node]={'error':repr(e)}
    write('live_coverage.json',{'wall':time.time(),'topics':coverage,'recorder_bus_stats':buses})
    live='/mppi/status' in pubs and '/joint_states_single' in pubs
    robot_live_seen|=live
    if live and not startup_captured:
     snapshot_runtime('startup',state);startup_captured=True
    if live and now-last_capture>30:
     snapshot_runtime('latest',state);last_capture=now
   except Exception as e:alerts.append('master_probe:'+repr(e))
  if mem.get('MemAvailable',0)<768*1024:reason='capture_memory_floor';running=False
  if sum(int(q.get('status',{}).get('VmRSS','0 kB').split()[0]) for q in rec)>512*1024:reason='capture_RSS_above_512MiB';running=False
  if os.statvfs(out).f_bavail*os.statvfs(out).f_frsize<2*1024**3:reason='capture_disk_floor_2GiB';running=False
  try:
   st=json.loads((out/'map_observer_status.json').read_text())
   if st.get('errors'):alerts.append('map_observer_errors')
  except (OSError,ValueError):pass
  if any(q['process'].poll() is not None for q in children):alerts.append('capture_incomplete_do_not_start_new_phase')
  write('status.json',{'wall':time.time(),'elapsed_s':now-beg,'running':running,'reason':reason,'recorders':rec,'bag_sizes':sizes,'alerts':alerts,'robot_live_seen':robot_live_seen,'capture_only':True})
  time.sleep(1)
finally:
 for q in children:
  if q['process'].poll() is None:os.killpg(q['pid'],signal.SIGINT)
 for q in children:
  try:q['process'].wait(timeout=15)
  except subprocess.TimeoutExpired:
   # No SIGKILL: retain active bag for recovery, and report it explicitly.
   pass
  q['log_handle'].close()
 resources.close();write('recording_closed.json',{'wall':time.time(),'reason':reason or 'requested_stop_or_600s_limit','recorders':[{'tag':q['tag'],'pid':q['pid'],'exit_code':q['process'].poll()} for q in children],'hardware_signalled':False})
