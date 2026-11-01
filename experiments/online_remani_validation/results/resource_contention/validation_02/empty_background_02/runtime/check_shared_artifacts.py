#!/usr/bin/env python3
"""Decode every newly generated image without modifying or following controls."""
import argparse
import hashlib
import json
from pathlib import Path

from PIL import Image


def check(root, output):
    root, output = Path(root).resolve(), Path(output).resolve()
    if output.exists():
        raise FileExistsError('Refusing to overwrite artifact audit: ' + str(output))
    if not root.is_dir():
        raise ValueError('Result directory not found: ' + str(root))
    rows, errors = [], []
    # rglob does not traverse symlinked trial directories. Explicitly reject
    # any symlink ancestor too, so old campaign controls stay read-only.
    for path in sorted(root.rglob('*')):
        if path.suffix.lower() not in ('.png', '.gif') or not path.is_file():
            continue
        if path.is_symlink() or any(p.is_symlink() for p in path.parents if p != root):
            continue
        try:
            with Image.open(path) as im:
                size, frames = list(im.size), getattr(im, 'n_frames', 1)
                for index in range(frames):
                    im.seek(index)
                    im.convert('RGB').load()
            rows.append(dict(path=path.relative_to(root).as_posix(), frames=frames,
                             size=size, sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
        except Exception as exc:
            errors.append(dict(path=str(path), error=str(exc)))
    report = dict(passed=bool(rows) and not errors,
                  scope='Generated images only; decoding is not algorithm or safety validation.',
                  result_directory=str(root), images=len(rows),
                  gifs=sum(r['path'].endswith('.gif') for r in rows),
                  gif_frames=sum(r['frames'] for r in rows if r['path'].endswith('.gif')),
                  records=rows, errors=errors)
    output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({k: v for k, v in report.items() if k != 'records'}, indent=2))
    if not report['passed']:
        raise RuntimeError('Artifact decoding failed; see ' + str(output))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    a = p.parse_args()
    check(a.root, a.out)
