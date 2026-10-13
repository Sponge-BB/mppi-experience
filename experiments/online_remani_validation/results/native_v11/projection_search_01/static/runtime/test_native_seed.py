#!/usr/bin/env python3
"""Link the real native search object into a small invalid-seed regression."""
import argparse
import json
from pathlib import Path
import shlex
import shutil
import subprocess
from build_native_v10 import compile_source
from build_native_v9 import sha

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args();build=args.build.resolve();out=args.out.resolve()
    manifest=json.loads((build/'build_manifest.json').read_text())
    parent=Path(manifest['parent_build'])
    assert sha(build/'source/sample_mani_RRT.cpp')==manifest['isolated_source_sha256']['sample_mani_RRT.cpp']
    (out/'build').mkdir(parents=True,exist_ok=False)
    link=shlex.split((parent/'build/CMakeFiles/remani_planner_node.dir/link.txt').read_text())
    old=next(s for s in link if s.endswith('/remani_planner_node.cpp.o'))
    _,main=compile_source(parent,out,'remani_planner_node',Path(__file__).with_suffix('.cpp'),old)
    _,sample=compile_source(parent,out,'exp_search',build/'source/sample_mani_RRT.cpp','unused')
    archive=out/'build/libexp_search.a';shutil.copy2(parent/'build/libexp_search.a',archive)
    subprocess.run(['ar','r',str(archive),sample],check=True)
    link=[main if s==old else str(archive) if s=='libexp_search.a' else s for s in link]
    binary=out/'build/test_native_seed';link[link.index('-o')+1]=str(binary)
    subprocess.run(link,cwd=str(parent/'build'),check=True)
    result=subprocess.run([str(binary)],capture_output=True,text=True,check=True,timeout=10)
    (out/'result.txt').write_text(result.stdout+result.stderr)
    print(result.stdout,result.stderr)
