#!/usr/bin/python3
import json
from pathlib import Path
root=Path(__file__).resolve().parent
rows=[];seen=set()
for name in ('active1','active2','post_cancel'):
 for line in (root/f'map_trace_events_{name}.jsonl').read_text().splitlines():
  r=json.loads(line);key=(r['t'],r['stage'])
  if key not in seen: rows.append(r);seen.add(key)
rows.sort(key=lambda r:r['t'])
gates=[]
for line in (root/'mppi_gate_rows.jsonl').read_text().splitlines():
 r=json.loads(line)
 for name in ('capture','checked'):
  g=r.get(name) or {};h=g.get('snapshot_header',[])
  if h:
   gates.append({'phase':name,'state':r['state'],'checked_at':g['checked_at'],
                 'wire':h[2],'field':h[14],'obs':h[29],'fusion':h[3],
                 'callback':g.get('snapshot_callback',{})})
result=[]
for abort in (1794632723.7447,1794632731.3708):
 def latest(stage,predicate=lambda _:True):
  options=[r for r in rows if r['stage']==stage and r['t']<=abort and predicate(r)]
  return options[-1] if options else None
 read=latest('read');complete=latest('read',lambda r:bool(r['complete']))
 near=[r for r in rows if abort-.17<=r['t']<=abort+.12 and r['stage'] in ('read','commit','export_sample','fusion')]
 relevant=[g for g in gates if abort-.15<=g['checked_at']<=abort+.05]
 result.append({'abort':abort,'last_read':read,'last_complete_read':complete,
                'last_commit':latest('commit'),'last_export_sample':latest('export_sample'),
                'relative_last_read_ms':(read['t']-abort)*1000,
                'relative_last_complete_read_ms':(complete['t']-abort)*1000,
                'nearby_map_events':near,'nearby_mppi_checks':relevant})
out={'events':result,'conclusion':[
 'First ABORT: complete obs907 was published about3.871ms before ABORT; MPPI had obs906. This proves export/adoption lag but does not prove slow safety used incomplete occupancy.',
 'Second ABORT: latest read had obs961/esdf960/complete0, published12.306ms before ABORT; complete obs961 was published71.900ms after ABORT. MPPI had obs960. Different-generation inputs were available during the safety decision.',
 'No log identifies the slow safety ReadScope acquisition time or bound observation/revision. Its actual generation and whether the checked sphere clearance differed across generations remain unproven.',
 'Post-stop same-generation raw/ESDF agreement is compatible with temporary generation lag; neither establishes actual cancellation causality.'
], 'source':'Existing MapTrace JSONL plus existing mppi_gate_rows.jsonl only; no bag reread or live observation.'}
(root/'abort_map_generation_alignment.json').write_text(json.dumps(out,indent=2)+'\n')
print(json.dumps(out,indent=2))
