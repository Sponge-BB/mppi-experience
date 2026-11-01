#!/usr/bin/env python3
"""Offline verification of this experiment's saved evidence, not safety proof."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path

from report_collab_factorial import read, sha, paired_measure
from audit_dual_timescale import build_integrity


def recorded_sha(path):
    if path.exists():return sha(path)
    archive=path.with_name(path.name+'.gz')
    h=hashlib.sha256()
    with gzip.open(archive,'rb') as stream:
        for chunk in iter(lambda:stream.read(1024*1024),b''):h.update(chunk)
    return h.hexdigest()


def verify(report,extra_batches):
    from PIL import Image
    summary=read(report/'summary.json');errors=[];trials=[];builds={};archives=0
    paths={Path(r['path']) for r in summary['online']+summary['paired']}
    for batch in extra_batches:
        if not read(batch/'batch_complete.json',{}).get('complete'):errors.append('incomplete extra batch '+str(batch))
        paths.update(p.parent for p in batch.glob('*/run_status.json'))
    for batch in {p.parent for p in paths}:
        plan=read(batch/'experiment_plan.json');expected={(s,g,n) for s in plan['scenes'] for g in plan['groups'] for n in plan['seeds']}
        actual=set()
        for p in batch.glob('*/run_status.json'):
            m=read(p.parent/'manifest.json');actual.add((m['scene'],m['factorial']['group'],m['factorial']['seed']))
        if actual!=expected:errors.append('trial cells missing/duplicated '+str(batch))
        if not read(batch/'batch_complete.json',{}).get('complete'):errors.append('incomplete batch '+str(batch))
    for path in sorted(paths):
        local=[];manifest=read(path/'manifest.json');build=Path(manifest['build'])
        if str(build) not in builds:builds[str(build)]=build_integrity(build,{})
        if not builds[str(build)]['pass_integrity']:local.append('build/production integrity')
        for name,digest in read(build/'build_manifest.json').get('isolated_source_sha256',{}).items():
            if sha(build/'source'/name)!=digest:local.append('isolated source changed '+name)
        if manifest['native_binaries_sha256']!=builds[str(build)]['native_binaries_sha256']:local.append('binary identity')
        if manifest['mppi_library_sha256']!=builds[str(build)]['mppi_library_sha256']:local.append('kernel identity')
        if manifest['mppi_library_sha256']!='ccfa50e5fcdfcab9e4519d7fd80d29c30b5ebbd29c4313fcad50c1900981f19d':local.append('not the frozen V12 kernel')
        for name,digest in manifest.get('experiment_sources_sha256',{}).items():
            candidate=path/'runtime'/name
            if Path(name).suffix=='.py' and (not candidate.exists() or sha(candidate)!=digest):local.append('runtime changed/missing '+name)
        for name,record in read(path/'compression_audit.json',{}).items():
            archive=path/(name+'.gz');archives+=1
            if not archive.exists() or sha(archive)!=record['gzip_sha256']:local.append('gzip archive '+name);continue
            if recorded_sha(path/name)!=record['original_sha256']:local.append('gzip original '+name)
        if manifest['factorial']['paired_diagnostic']:
            _,audit=paired_measure(path)
            if not audit['valid']:local+=audit['errors']
        else:
            audit=read(path/'trial_audit.json')
            if not audit['valid_experiment']:local+=audit['errors']
            for name,digest in audit['inputs_sha256'].items():
                if recorded_sha(path/name)!=digest:local.append('audited evidence changed '+name)
        trials.append(dict(path=str(path),valid=not local,errors=local))
        errors.extend(str(path)+': '+s for s in local)
    visuals=[]
    for name in summary['gifs']:
        path=report/name
        with Image.open(path) as animation:
            frames=animation.n_frames
            for i in range(frames):animation.seek(i);animation.load()
            size=animation.size
        audit=read(path.parent/'visualization_audit.json');local=[]
        if frames!=audit['frames'] or frames<2:local.append('GIF frame count')
        for source in audit['sources']:
            root=Path(source['path'])
            for item,digest in source['hashes'].items():
                if recorded_sha(root/item)!=digest:local.append('visual source changed '+item)
        with Image.open(path.parent/'final_paths.png') as still:still.load()
        visuals.append(dict(path=str(path),frames=frames,size=size,valid=not local,errors=local))
        errors+=local
    for path in report.glob('*.png'):
        with Image.open(path) as chart:chart.load()
    for key in ('all_online_integrity','all_paired_integrity','all_wheel_bounds'):
        if not summary[key]:errors.append('summary check failed: '+key)
    if not summary['common_inputs_audit']['valid']:errors.append('common inputs')
    if summary['production_changed']:errors.append('production files changed')
    result=dict(valid=not errors,errors=errors,scope='evidence integrity only; navigation failure is retained, not invalidated',
        trials=trials,verified_archives=archives,builds=builds,visuals=visuals,
        report_files_sha256={str(p.relative_to(report)):sha(p) for p in report.rglob('*')
                             if p.is_file() and p.name!='artifact_integrity.json'})
    (report/'artifact_integrity.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
    print('ARTIFACTS',result['valid'],'trials',len(trials),'archives',archives,'GIF',len(visuals),flush=True)
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--report',type=Path,required=True)
    p.add_argument('--extra-batch',type=Path,action='append',default=[]);a=p.parse_args()
    raise SystemExit(0 if verify(a.report,a.extra_batch)['valid'] else 1)
