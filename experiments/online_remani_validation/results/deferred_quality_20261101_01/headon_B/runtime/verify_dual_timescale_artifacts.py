#!/usr/bin/env python3
"""Final read-only provenance and complete image-decoding checks."""
import argparse
import hashlib
import json
from pathlib import Path

from PIL import Image

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verify(base):
    base=base.resolve();videos=[];figures=[]
    expected=json.loads((HERE/'builds/dual_timeline_v12_01/build_manifest.json').read_text())['production_sources_sha256']
    changed=[name for name,digest in expected.items() if sha(ROOT/name)!=digest]
    assert not changed,changed
    assert json.loads((base/'online_audit_02.json').read_text())['all_valid']
    assert json.loads((base/'delivery_audit_02.json').read_text())['all_valid']
    local=json.loads((base/'local_rejoin_summary.json').read_text())
    assert local['trials']==18 and local['passed']==17
    assert local['confirmation']['same_seed_state_and_controls_identical']
    for audit in sorted((base/'figures').rglob('visualization_audit.json')):
        data=json.loads(audit.read_text());gif=audit.parent/'overview.gif'
        assert data['source_unchanged'] and data['no_future_reference'] and data['no_missing_rollout_fabricated']
        for source in data['sources']:
            for name,digest in source['hashes'].items():
                assert sha(Path(source['path'])/name)==digest,(audit,name)
        frames=0;duration=0
        with Image.open(gif) as im:
            size=im.size
            for index in range(im.n_frames):
                im.seek(index);im.load();frames+=1;duration+=im.info.get('duration',0)
        assert frames>1
        videos.append(dict(path=str(gif.relative_to(base)),sha256=sha(gif),bytes=gif.stat().st_size,
            frames_fully_decoded=frames,size=list(size),duration_ms=duration,
            declared_playback_multiplier=data['actual_regular_playback_multiplier']))
    assert len(videos)==33,len(videos)
    index=(base/'GIF_INDEX.md').read_text()
    for video in videos:assert video['path'] in index
    for path in sorted((base/'figures').rglob('*.png'))+sorted(base.glob('*.png')):
        with Image.open(path) as im:im.load();size=im.size
        figures.append(dict(path=str(path.relative_to(base)),sha256=sha(path),bytes=path.stat().st_size,size=list(size)))
    texts=[]
    for path in (base/'README.md',base/'GIF_INDEX.md',base/'实验结果与下一步建议.txt'):
        assert path.read_text().strip()
        texts.append(dict(path=path.name,sha256=sha(path),bytes=path.stat().st_size))
    report=dict(verified=True,date='2026-10-18 Asia/Shanghai',
        production_sources_verified=len(expected),production_sources_unchanged=True,
        online_trials=11,fixed_reference_fixtures=18,additional_same_input_confirmation=1,
        gif_count=len(videos),png_count=len(figures),
        videos=videos,figures=figures,texts=texts,
        scope='Artifact decoding and provenance; not an algorithm or physical safety certificate')
    (base/'artifact_integrity.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
    print('ARTIFACTS_VERIFIED',len(videos),'GIF',len(figures),'PNG',len(expected),'production files',flush=True)
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out',type=Path,default=HERE/'results/dual_timescale_20261018')
    verify(p.parse_args().out)
