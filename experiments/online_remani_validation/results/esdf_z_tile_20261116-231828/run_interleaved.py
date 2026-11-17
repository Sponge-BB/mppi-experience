#!/usr/bin/env python3
"""Paired serial runs of the unchanged historical dense_worker_benchmark.
One warmup pair plus5 measured pairs; only the final pair of raw fields retained.
"""
import os
os.environ.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MALLOC_ARENA_MAX='2')
import argparse,hashlib,json,statistics,subprocess,resource,time
from pathlib import Path
import numpy as np
CASE=Path(__file__).resolve().parent

def sha(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
 return h.hexdigest()

def verify(p):
 for line in p.read_text().splitlines():
  digest,name=line.split(maxsplit=1)
  if sha(Path(name))!=digest:raise RuntimeError('Hash changed:'+name)

def stats(values):
 a=sorted(values)
 return {'n':len(a),'median':statistics.median(a),'min':a[0],'max':a[-1]}

def compare_outputs(run,frames):
 results=[]
 for f in frames:
  paths=[run/v/(f['name']+'.distance.f64') for v in ['baseline','z_tile8']]
  n=f['voxels']
  if any(p.stat().st_size!=n*8 for p in paths):raise RuntimeError('Output size mismatch')
  x,y=[np.memmap(p,dtype=np.float64,mode='r',shape=(n,)) for p in paths]
  mismatch=nonfinite=0;maxdiff=0.
  for start in range(0,n,65536):
   a,b=x[start:start+65536],y[start:start+65536]
   mismatch+=int(np.count_nonzero(a!=b));nonfinite+=int(np.count_nonzero(~np.isfinite(a)|~np.isfinite(b)))
   maxdiff=max(maxdiff,float(np.max(np.abs(a-b))))
  results.append({'frame':f['name'],'group':f['group'],'mismatch_count':mismatch,'nonfinite_count':nonfinite,'max_abs_difference':maxdiff,'baseline_sha256':sha(paths[0]),'candidate_sha256':sha(paths[1])})
  del x,y,a,b
 return results

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--threads',type=int,choices=(1,2,3),default=3);ap.add_argument('--pairs',type=int,default=5);args=ap.parse_args()
 if args.pairs<5:raise ValueError('At least5 measured pairs required')
 run=CASE/('paired_t%d'%args.threads);run.mkdir(exist_ok=False)
 for v in ['baseline','z_tile8']:(run/v).mkdir()
 fixtures=CASE/'fixtures';metadata=json.load(open(fixtures/'metadata.json'));frames=metadata['frames']
 for f in frames:
  if sha(fixtures/f['file'])!=f['sha256']:raise RuntimeError('Fixture hash mismatch')
 rows=[];comparisons=[];t0=time.monotonic()
 for round_index in range(args.pairs+1):
  order=['baseline','z_tile8'] if round_index%2==0 else ['z_tile8','baseline']
  for variant in order:
   verify(CASE/'build'/(variant+'.inputs.after.sha256'));verify(CASE/'build'/(variant+'.binary.sha256'))
   with (run/('round%02d_%s.jsonl'%(round_index,variant))).open('w') as out,(run/('round%02d_%s.stderr'%(round_index,variant))).open('w') as err:
    command=['nice','-n','10',str(CASE/'build'/(variant+'-worker-benchmark')),str(fixtures),'1',str(args.threads),str(run/variant)]
    subprocess.run(command,stdout=out,stderr=err,check=True,timeout=180)
   samples=[json.loads(line) for line in (run/('round%02d_%s.jsonl'%(round_index,variant))).read_text().splitlines()]
   if len(samples)!=len(frames):raise RuntimeError('Incomplete benchmark round')
   for row in samples:row.update(round=round_index,warmup=round_index==0,order=order.index(variant));rows.append(row)
  exact=compare_outputs(run,frames);comparisons.append({'round':round_index,'warmup':round_index==0,'frames':exact})
  (run/'comparisons.json').write_text(json.dumps(comparisons,indent=2)+'\n')
  if any(r['mismatch_count'] or r['nonfinite_count'] for r in exact):raise RuntimeError('Non-exact output; timing not accepted')
  print(json.dumps({'round':round_index,'warmup':round_index==0,'exact':True,'order':order}),flush=True)
 per_frame=[]
 for f in frames:
  variants={}
  for v in ['baseline','z_tile8']:
   samples=[r for r in rows if r['frame']==f['name'] and r['variant']==v and not r['warmup']]
   variants[v]={k:stats([r[k] for r in samples]) for k in ['compute_ms','submit_wait_ms','queue_wait_ms','prepare_ms','near_free_ms','negative_apply_ms']}
   variants[v]['positive_y_ms']=stats([r['positive_zyx_ms'][1] for r in samples]);variants[v]['positive_x_ms']=stats([r['positive_zyx_ms'][2] for r in samples])
   variants[v]['peak_rss_MiB']=max(r['peak_rss_kib'] for r in samples)/1024.
  baseline=variants['baseline']['compute_ms']['median'];candidate=variants['z_tile8']['compute_ms']['median']
  per_frame.append({'frame':f['name'],'group':f['group'],'voxels':f['voxels'],'variants':variants,'candidate_over_baseline':candidate/baseline,'reduction_percent':100*(baseline-candidate)/baseline})
 summary={'schema':'esdf_z_tile_paired_v1','threads':args.threads,'measured_pairs':args.pairs,'warmup_pairs':1,'alternating_order':True,'exact_every_pair':True,'per_frame':per_frame,'elapsed_s':time.monotonic()-t0,'analysis_peak_rss_MiB':resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024.,'worker_peak_rss_MiB':max(r['peak_rss_kib'] for r in rows)/1024.,'real_scope':metadata['real_scope'],'synthetic_scope':metadata['synthetic_scope'],'timing_scope':'worker only; no fusion/commit/export, ROS, concurrent planner/MPPI or original full real worker inputs','retained_raw_fields':'latest pair only; all pair comparisons/hashes retained'}
 (run/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');(run/'all_rows.json').write_text(json.dumps(rows,indent=2)+'\n')
 print(json.dumps({'summary':str(run/'summary.json'),'exact':True,'real_median_reduction_percent':statistics.median(r['reduction_percent'] for r in per_frame if r['group']=='real_crop'),'synthetic_reductions':{r['frame']:r['reduction_percent'] for r in per_frame if r['group']=='synthetic'}}))
if __name__=='__main__':main()
