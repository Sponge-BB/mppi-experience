import datetime,json,time,os
from pathlib import Path
meta=json.loads(Path('/tmp/remani_dynamic_capture_active.json').read_text());p=Path(meta['output'])
operator=[json.loads(s) for s in (p/'operator_events.jsonl').read_text().splitlines() if s.strip()]
threshold=operator[-1]['wall'];began=time.monotonic()
armed=dict(pid=os.getpid(),armed_wall=time.time(),goal_received_after_wall=threshold,wait_limit_s=300,trigger='next_operator_test_goal',control_publications=False)
(p/'dynamic_marker_armed.json').write_text(json.dumps(armed,indent=2)+'\n')
while time.monotonic()-began<300:
 try:
  status=json.loads((p/'status.json').read_text())
  if status['phase']=='closed':break
  events=[json.loads(s) for s in (p/'goal_events.jsonl').read_text().splitlines() if s.strip()]
  candidates=[e for e in events if e['received_wall']>=threshold]
  if candidates:
   marker=dict(wall=time.time(),local=datetime.datetime.now().astimezone().isoformat(),goal=candidates[0],phase='dynamic_test_goal_received',operator_observation='Transit navigation stuttered before deliberate obstacle test.',control_publications=False)
   dest=p/'dynamic_test_started.json';tmp=p/'dynamic_test_started.tmp';tmp.write_text(json.dumps(marker,ensure_ascii=False,indent=2)+'\n');tmp.replace(dest)
   print(json.dumps(marker,ensure_ascii=False),flush=True);break
 except (OSError,ValueError):pass
 time.sleep(.2)
else:
 (p/'dynamic_marker_wait_expired.json').write_text(json.dumps(dict(wall=time.time(),reason='No new test goal within 300 seconds. Main recorder has its own limits.'),indent=2)+'\n')
