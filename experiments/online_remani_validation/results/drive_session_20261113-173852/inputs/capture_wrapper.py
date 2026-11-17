import hashlib,importlib.util,json,os,signal,struct,threading
from pathlib import Path
import rospy
root=Path('/home/l/remani_ws');version='20261113-172845-map-latency'
spec=importlib.util.spec_from_file_location('drive_recorder',root/'experiments/online_remani_validation/record_drive_session.py');module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
class MapValidationRecorder(module.Recorder):
 def __init__(self):
  self.header_log=None;self.library_mapped=False
  super().__init__()
  self.header_log=(self.out/'cloud_headers.jsonl').open('w')
  manifest=json.loads((root/'rollback'/version/'manifest.json').read_text());files=json.loads((root/'rollback'/version/'files.json').read_text());verified={}
  for n,v in files.items():
   actual=hashlib.sha256((root/n).read_bytes()).hexdigest();assert actual==v['sha256'],'code changed: '+n;verified[n]=actual
  (self.out/'deployed_version.json').write_text(json.dumps({'version':version,'parent':manifest['parent_version'],'files_sha256':verified,'deployment':manifest['deployment'],'hardware_motion_sent_by_recorder':False},ensure_ascii=False,indent=2)+'\n')
  (self.out/'inputs/capture_wrapper.py').write_bytes(Path(__file__).read_bytes())
  Path('/tmp/remani_active_map_capture.json').write_text(json.dumps({'output':str(self.out),'version':version,'pid':os.getpid(),'phase':'waiting_for_startup','mode':'read-only subscriptions; no enable/reset/goal/motion publication'},ensure_ascii=False,indent=2)+'\n')
  print('MAP_VALIDATION_READY '+str(self.out),flush=True)
 def receive(self,msg,topic):
  received=rospy.get_time()
  if topic in getattr(self,'cloud_topics',()) and self.header_log is not None:
   raw=msg._buff;row={'topic':topic,'receive_ros':received,'bytes':len(raw)}
   try:
    seq,sec,nsec=struct.unpack_from('<III',raw);row.update(seq=seq,source_sec=sec,source_nsec=nsec,source_stamp=sec+nsec*1e-9)
   except struct.error as e:row['error']=str(e)
   with self.lock:
    if not self.done:self.append_json(self.header_log,row)
  super().receive(msg,topic)
 def status(self,memory):
  if not self.library_mapped:
   target=root/'devel/lib/libplan_env.so'
   for p in Path('/proc').glob('[0-9]*/cmdline'):
    try:
     if str(root/'devel/lib/remani_planner/remani_planner_node').encode() not in p.read_bytes().split(b'\0'):continue
     mapped=[]
     for line in (p.parent/'maps').read_text().splitlines():
      if '/libplan_env.so' in line:
       f=line.split();mapped.append({'path':' '.join(f[5:]),'inode':int(f[4]),'matches_verified_deployed_inode':int(f[4])==target.stat().st_ino and f[-1]==str(target)})
     if mapped:
      (self.out/'runtime_library.json').write_text(json.dumps({'planner_pid':int(p.parent.name),'mapped_library':mapped,'verified_version':version,'recorded_at_ros':rospy.get_time()},indent=2)+'\n');self.library_mapped=True;break
    except OSError:pass
  if self.header_log is not None:self.header_log.flush()
  r=super().status(memory);r['version']=version;r['runtime_library_captured']=self.library_mapped;r['cloud_headers']='all received callbacks; raw payloads remain bounded/sampled';return r
 def run(self):
  try:super().run()
  finally:
   if self.header_log is not None:self.header_log.close()
rospy.init_node('map_validation_readonly',anonymous=True,disable_rosout=True,disable_signals=True)
recorder=MapValidationRecorder()
for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda *_:recorder.stop('user_interrupt'))
recorder.run()
