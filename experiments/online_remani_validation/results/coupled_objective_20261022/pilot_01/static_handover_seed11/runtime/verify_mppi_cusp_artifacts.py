#!/usr/bin/env python3
"""Verify derived images/GIF decoding and retained experiment provenance."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil

from PIL import Image

from run_mppi_cusp_adjustment import HERE,BUILDS,verify_build


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--root',type=Path,required=True);a=p.parse_args()
    root=a.root.resolve();summary=json.loads((root/'summary_audit.json').read_text())
    for name,build in BUILDS.items():
        if verify_build(build)!=summary['builds'][name]:raise ValueError('changed experiment build '+name)
    production=json.loads((BUILDS['baseline']/'build_manifest.json').read_text())['production_sources_sha256']
    for name,digest in production.items():
        if sha(HERE.parents[1]/name)!=digest:raise ValueError('changed production '+name)
    snapshots=[]
    for suite in ('focus_02','regression_01','gated_validation_01','gated_objective_validation_01','dynamic_seeds_01','reset_control_01'):
        out=root/suite;protocol=json.loads((out/'protocol.json').read_text())
        if sha(out/'runner_snapshot.py')!=protocol['script_sha256']:raise ValueError('runner start mismatch '+suite)
        if 'runner_sha256' in protocol:
            fixture=out/'fixture_runner_snapshot.py'
            if not fixture.exists():
                source=HERE/'run_mppi_rejoin_diagnosis.py'
                if sha(source)!=protocol['runner_sha256']:raise ValueError('cannot recover exact fixture runner '+suite)
                shutil.copy2(source,fixture)
            if sha(fixture)!=protocol['runner_sha256']:raise ValueError('fixture runner mismatch '+suite)
        snapshots.append(dict(suite=suite,runner_sha256=protocol['script_sha256']))
    # Every visual input, including real exported controls/search, is hash
    # checked. Source records are not amended or replaced by reconstructed data.
    hashes={};videos=[];pngs=[]
    for audit_path in sorted((root/'figures').glob('*/visualization_audit.json')):
        audit=json.loads(audit_path.read_text())
        for source in audit['sources']:
            for relative,expected in source['hashes'].items():
                path=Path(source['path'])/relative
                if path not in hashes:hashes[path]=sha(path)
                if hashes[path]!=expected:raise ValueError('visual source changed '+str(path))
        path=audit_path.parent/'overview.gif'
        with Image.open(path) as im:
            count=0
            for frame in range(im.n_frames):im.seek(frame);im.load();count+=1
            if count!=audit['frames']:raise ValueError('GIF frame count mismatch')
        videos.append(dict(path=str(path.relative_to(root)),sha256=sha(path),decoded_frames=count))
    if len(videos)!=8:raise ValueError('expected eight categorized GIFs')
    for path in root.rglob('*.png'):
        with Image.open(path) as im:im.load()
        pngs.append(dict(path=str(path.relative_to(root)),sha256=sha(path)))
    report=dict(verified=True,date='2026-10-18 Asia/Shanghai',production_sources_verified=len(production),
        production_sources_unchanged=True,gif_count=len(videos),png_count=len(pngs),videos=videos,pngs=pngs,
        runner_snapshots=snapshots,visual_source_files_verified=len(hashes),
        selected_variant=summary['variants']['gated_objective'],
        scope='Controlled numerical fixed-reference fixtures; no online REMANI/Gazebo/real-vehicle validation')
    (root/'artifact_integrity.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
    print('CUSP_ARTIFACTS_VERIFIED',len(videos),'GIF',len(pngs),'PNG',len(production),'production files',flush=True)


if __name__=='__main__':main()
