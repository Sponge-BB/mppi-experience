#!/usr/bin/python3
import os
os.environ.update(OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', MALLOC_ARENA_MAX='2')
import argparse, hashlib, json, resource, statistics, subprocess, time
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
        h,name=line.split(maxsplit=1)
        if sha(Path(name))!=h:raise RuntimeError('Hash mismatch: '+name)
def stat(v):
    a=np.asarray(v,dtype=float)
    return {'n':len(v),'median':float(np.median(a)),'p95':float(np.quantile(a,.95)),'min':min(v),'max':max(v)}

ap=argparse.ArgumentParser();ap.add_argument('candidate',choices=['axis_layout','negative_subset','combined','float32_scratch']);ap.add_argument('--pairs',type=int,default=5);ap.add_argument('--threads',type=int,choices=[1,2,3],default=3);args=ap.parse_args()
if args.pairs<5:raise ValueError('Need at least five measured pairs')
resource.setrlimit(resource.RLIMIT_AS,(600*1024**2,600*1024**2))
variants=['baseline',args.candidate];fixtures=CASE/'fixtures';meta=json.loads((fixtures/'metadata.json').read_text());frames=meta['frames']
for f in frames:
    if sha(fixtures/f['file'])!=f['sha256']:raise RuntimeError('Fixture mismatch')
run=CASE/('paired_'+args.candidate+'_t'+str(args.threads));run.mkdir(exist_ok=False)
for v in variants:(run/v).mkdir()
allrows=[];comparisons=[];t0=time.monotonic()
for n in range(args.pairs+1):
    order=variants if n%2==0 else variants[::-1]
    for v in order:
        verify(CASE/'build'/(v+'.inputs.after.sha256'));verify(CASE/'build'/(v+'.binary.sha256'))
        with (run/('round%02d_%s.jsonl'%(n,v))).open('w') as out,(run/('round%02d_%s.stderr'%(n,v))).open('w') as err:
            subprocess.run(['nice','-n','10',str(CASE/'build'/(v+'-worker-benchmark')),str(fixtures),'1',str(args.threads),str(run/v)],stdout=out,stderr=err,check=True,timeout=180)
        rows=[json.loads(l) for l in (run/('round%02d_%s.jsonl'%(n,v))).read_text().splitlines()]
        if len(rows)!=len(frames):raise RuntimeError('Incomplete benchmark')
        for r in rows:r.update(round=n,warmup=n==0,order=order.index(v));allrows.append(r)
    results=[]
    for f in frames:
        files=[run/v/(f['name']+'.distance.f64') for v in variants];count=f['voxels']
        if any(p.stat().st_size!=count*8 for p in files):raise RuntimeError('Wrong field size')
        x,y=[np.memmap(p,dtype=np.float64,mode='r',shape=(count,)) for p in files]
        mismatch=nonfinite=0;maxdiff=0.
        for start in range(0,count,65536):
            a,b=x[start:start+65536],y[start:start+65536]
            mismatch+=int(np.count_nonzero(a!=b));nonfinite+=int(np.count_nonzero(~np.isfinite(a)|~np.isfinite(b)))
            maxdiff=max(maxdiff,float(np.max(np.abs(a-b))))
        results.append({'frame':f['name'],'group':f['group'],'mismatch_count':mismatch,'nonfinite_count':nonfinite,'max_abs_diff':maxdiff,'sha256':{v:sha(p) for v,p in zip(variants,files)}})
        del x,y,a,b
    comparisons.append({'round':n,'warmup':n==0,'frames':results})
    (run/'comparisons.json').write_text(json.dumps(comparisons,indent=2)+'\n')
    if any(r['mismatch_count'] or r['nonfinite_count'] for r in results):raise RuntimeError('Non-exact field')
    # These are transient outputs created only inside this fresh run directory.
    # Preserve every comparison/hash; retain two representative final fields.
    for f in frames:
        if n==args.pairs and f['name'] in ['real_00','synthetic_subset_sparse']:continue
        for v in variants:(run/v/(f['name']+'.distance.f64')).unlink()
    print(json.dumps({'round':n,'exact':True,'order':order}),flush=True)
summary=[]
for f in frames:
    vstats={}
    for v in variants:
        rows=[r for r in allrows if r['frame']==f['name'] and r['variant']==v and not r['warmup']]
        vals={k:stat([r[k] for r in rows]) for k in ['compute_ms','submit_wait_ms','queue_wait_ms','near_free_ms','negative_apply_ms']}
        for sign in ['positive','negative']:
            for i,axis in enumerate('zyx'):vals[sign+'_'+axis+'_ms']=stat([r[sign+'_zyx_ms'][i] for r in rows])
        vals['peak_rss_MiB']=max(r['peak_rss_kib'] for r in rows)/1024
        vals['negative_fallback_count']=sorted(set(r['negative_fallback_count'] for r in rows))
        if 'exact_float_scratch_count' in rows[0]:vals['exact_float_scratch_count']=sorted(set(r['exact_float_scratch_count'] for r in rows))
        if 'negative_subset_cells' in rows[0]:
            for k in ['negative_subset_count','negative_subset_cells','negative_subset_z_layers','negative_subset_yz_lines']:vals[k]=sorted(set(r[k] for r in rows))
        vstats[v]=vals
    before=vstats['baseline']['compute_ms']['median'];after=vstats[args.candidate]['compute_ms']['median']
    summary.append({'frame':f['name'],'group':f['group'],'voxels':f['voxels'],'reduction_percent':100*(before-after)/before,'variants':vstats})
report={'candidate':args.candidate,'threads':args.threads,'pairs':args.pairs,'exact_every_pair':True,'elapsed_s':time.monotonic()-t0,'analysis_peak_rss_MiB':resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024,'worker_peak_rss_MiB':max(r['peak_rss_kib'] for r in allrows)/1024,'scope':meta['scope'],'per_frame':summary,'real_crop_median_reduction_percent':statistics.median(r['reduction_percent'] for r in summary if r['group']=='real_crop'),'synthetic_reductions':{r['frame']:r['reduction_percent'] for r in summary if r['group']!='real_crop'}}
(run/'summary.json').write_text(json.dumps(report,indent=2)+'\n');(run/'all_rows.json').write_text(json.dumps(allrows,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k!='per_frame'}),flush=True)
