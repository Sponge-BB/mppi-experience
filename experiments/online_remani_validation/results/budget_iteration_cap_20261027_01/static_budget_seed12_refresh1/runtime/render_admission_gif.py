#!/usr/bin/env python3
"""Read-only two-panel GIF; transparently read lossless archived recorder files."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path

import render_dual_timescale as renderer
from report_domain_recovery import assert_plot_quiet


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--off',type=Path,required=True);parser.add_argument('--on',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True);args=parser.parse_args()
    sources=[args.off.resolve(),args.on.resolve()];out=args.out.resolve()
    if out.exists() or any(out==p or out in p.parents for p in sources):parser.error('Use a fresh derived output directory')
    statuses=[dict(run_status=json.loads((p/'run_status.json').read_text())) for p in sources]
    assert_plot_quiet(dict(trials=statuses))
    originals={};read_json=renderer.read_json;read_jsonl=renderer.read_jsonl
    def archived(path,lines=False,default=None):
        path=Path(path);target=path if path.exists() else path.with_name(path.name+'.gz')
        if not target.exists():return [] if lines else default
        originals[str(target)]=hashlib.sha256(target.read_bytes()).hexdigest()
        with (gzip.open(target,'rt') if target.suffix=='.gz' else target.open()) as stream:
            return [json.loads(line) for line in stream if line.strip()] if lines else json.load(stream)
    renderer.read_json=lambda path,default=None:archived(path,default=default)
    renderer.read_jsonl=lambda path:archived(path,lines=True)
    try:
        result=renderer.render(sources,out,['起点预检关闭','起点预检开启'],playback=2.,step=.25)
    finally:
        renderer.read_json=read_json;renderer.read_jsonl=read_jsonl
    changed=[name for name,digest in originals.items() if hashlib.sha256(Path(name).read_bytes()).hexdigest()!=digest]
    if changed:raise RuntimeError('Renderer source changed: '+str(changed))
    result.update(read_archived_actual_records=True,archive_inputs_sha256=originals,
        source_unchanged=True,comparison='online closed loops, same seed but not identical time-varying inputs; no statistical performance claim')
    (out/'archive_read_audit.json').write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n')
    print(out)


if __name__=='__main__':main()
