from pathlib import Path
import io,struct,sys,json,hashlib,resource
from rospy.msg import deserialize_messages
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Float64MultiArray
P=Path(__file__).resolve().parents[1]
sys.path.insert(0,'/home/l/remani_ws/src/mm_remani/scripts/mppi')
from active_arm_mppi import Snapshot
old_raw=(P/'map_events/event08_status_referenced_full.msg').read_bytes()
full_raw=(P/'map_events/event09_status_referenced_full.msg').read_bytes()
delta_raw=(P/'map_events/event09_status_referenced_packet.msg').read_bytes()
data_class=numpy_msg(Float64MultiArray)
old=Snapshot.decode(data_class().deserialize(old_raw).data)
wire=struct.pack('<I',len(full_raw))+full_raw+struct.pack('<I',len(delta_raw))+delta_raw
results=[]
for size in (1,2):
 buffer=io.BytesIO(wire);buffer.seek(0,2);messages=[]
 deserialize_messages(buffer,messages,data_class,queue_size=size)
 current=old;steps=[]
 for m in messages:
  try:
   current=Snapshot.decode(m.data,current);steps.append({'seq':float(m.data[2]),'field':float(m.data[14]),'full':float(m.data[20]),'accepted':True})
  except ValueError as error:
   current=None;steps.append({'seq':float(m.data[2]),'field':float(m.data[14]),'full':float(m.data[20]),'accepted':False,'reason':str(error)})
 results.append({'queue_size':size,'deserialized_count':len(messages),'steps':steps})
assert len(results[0]['steps'])==1 and not results[0]['steps'][0]['accepted']
assert len(results[1]['steps'])==2 and all(x['accepted'] for x in results[1]['steps'])
report=json.loads((P/'analysis/first_fault_cache_protocol_audit.json').read_text())
report['actual_rospy_receive_queue_counterexample']={'runtime_function':'/opt/ros/noetic/lib/python3/dist-packages/rospy/msg.py:164-225 deserialize_messages','runtime_function_source_sha256':hashlib.sha256(Path('/opt/ros/noetic/lib/python3/dist-packages/rospy/msg.py').read_bytes()).hexdigest(),'data_class':'rospy.numpy_msg.numpy_msg(std_msgs.msg.Float64MultiArray)','input':'Actual saved matching full seq4121 + delta seq4122, each prefixed with little-endian uint32 payload length, concatenated in one BytesIO. Previous cache is actual old full seq4078/field1164.','batch_bytes':len(wire),'results':results,'proven':'For a valid full+delta receive batch, queue_size=1 deterministically prunes the prerequisite full before deserialization and gives the callback only an undecodable delta. queue_size=2 retains both and decoding succeeds. This is an incompatible latest-only queue and dependent-delta protocol.','not_proven':'Does not prove the exact wire4117 transport callback batch during the actual run. It proves a concrete failure mode in the actual ROS receive function with actual recorded payloads, without assuming network loss.','resource_peak_rss_mib':resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024}
(P/'analysis/first_fault_cache_protocol_audit.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print(json.dumps({'results':results,'rss':resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024}))
