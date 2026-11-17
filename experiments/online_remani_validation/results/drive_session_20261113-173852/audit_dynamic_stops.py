#!/usr/bin/python3
"""Read only closed-bag small topics; retain source/receipt time separately."""
import collections
import datetime
import json
from pathlib import Path
import resource

import rosbag

HERE = Path(__file__).resolve().parent
TZ = datetime.timezone(datetime.timedelta(hours=8))
topics = ['/rosout_agg', '/mppi/status', '/move_base_simple/goal',
          '/planning/trajectory', '/planning/safety_speed_limit',
          '/tracking_fault', '/dynamic_points', '/cmd_vel']
goals = []
counts = collections.Counter()
with (HERE/'dynamic_stop_small_topics.jsonl').open('w') as out, rosbag.Bag(str(HERE/'capture.bag')) as bag:
    for topic, msg, stamp in bag.read_messages(topics=topics):
        t = stamp.to_sec()
        row = dict(topic=topic, receipt=t,
                   local=datetime.datetime.fromtimestamp(t, TZ).isoformat(timespec='milliseconds'))
        counts[topic] += 1
        if topic == '/rosout_agg':
            if not any(word in msg.msg.lower() for word in (
                    'collision', '[fsm]', 'pointflow', 'point flow',
                    'dynamicpointflow', 'dynamictracker', 'tracking', 'abort',
                    'depth lost', 'emergency', 'mppi command rejected')):
                continue
            row.update(source=msg.header.stamp.to_sec(), node=msg.name,
                       level=msg.level, text=msg.msg)
        elif topic == '/mppi/status':
            data = json.loads(msg.data)
            diag = data.get('candidate_diagnostics', {})
            row['status'] = {k:v for k,v in data.items() if k != 'candidate_diagnostics'}
            if data.get('state') == 'no_feasible_candidate':
                row['diagnostics'] = diag
            else:
                row['diagnostics'] = {k:v for k,v in diag.items() if k in (
                    'mode','intervention','selection','selected_family','feasible',
                    'reference_status','reference_source','reference_wait_hold','hold_reason',
                    'external_min_gap','min_gap','structured_labels','structured_valid',
                    'structured_first_base_commands','structured_external_gaps',
                    'nominal_external_min_gap','reference_revision')}
        elif topic == '/planning/trajectory':
            row.update(source=msg.header.stamp.to_sec(), action=int(msg.action),
                       trajectory_id=int(msg.trajectory_id), blocks=len(msg.trajectory))
        elif topic == '/move_base_simple/goal':
            row.update(source=msg.header.stamp.to_sec(), frame=msg.header.frame_id,
                       x=msg.pose.position.x, y=msg.pose.position.y)
            goals.append(row.copy())
        elif topic == '/planning/safety_speed_limit' or topic == '/tracking_fault':
            row['data'] = msg.data
        elif topic == '/cmd_vel':
            row.update(v=msg.linear.x, w=msg.angular.z)
        elif topic == '/dynamic_points':
            row.update(source=msg.header.stamp.to_sec(),type=msg._type)
            if hasattr(msg, 'width'):
                row.update(points=int(msg.width*msg.height), fields=[f.name for f in msg.fields])
            else:
                row['data_length'] = len(msg.data)
        out.write(json.dumps(row, ensure_ascii=False, separators=(',',':'))+'\n')
summary = dict(goals=goals, topic_counts=dict(counts),
               peak_rss_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024.)
(HERE/'dynamic_stop_extraction.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(summary,ensure_ascii=False,indent=2))
