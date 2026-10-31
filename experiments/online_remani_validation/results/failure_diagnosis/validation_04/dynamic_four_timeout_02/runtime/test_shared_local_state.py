#!/usr/bin/env python3
"""Compile real controller state regressions with bounds assertions; no ROS."""
import argparse
import ctypes
import hashlib
import json
from pathlib import Path
import shlex
import shutil
import subprocess


def test(build,out):
    build=build.resolve();out=out.resolve();out.mkdir(parents=True,exist_ok=False)
    manifest=json.loads((build/'build_manifest.json').read_text())
    parent=Path(manifest['parent_build'])
    cm=parent/'build/CMakeFiles/online_mppi.dir'
    includes=next(s.split('=',1)[1] for s in (cm/'flags.make').read_text().splitlines() if s.startswith('CXX_INCLUDES ='))
    probe=out/'state_probe.cpp';shutil.copy2(Path(__file__).parent/'shared_local_state_probe.cpp',probe)
    obj=out/'state_probe.cpp.o'
    subprocess.run(['/usr/bin/c++','-std=gnu++17','-O1','-fPIC','-D_GLIBCXX_ASSERTIONS',
        '-I'+str(build/'source'),*shlex.split(includes),'-c',str(probe),'-o',str(obj)],check=True)
    link=shlex.split((cm/'link.txt').read_text())
    objects=[s for s in link if s.endswith('/online_api.cpp.o') or s=='online_api.cpp.o']
    assert len(objects)==1
    link=[str(obj) if s==objects[0] else s for s in link]
    library=out/'libstate_probe.so';link[link.index('-o')+1]=str(library)
    subprocess.run(link,cwd=str(parent/'build'),check=True)
    # Subprocess isolates an assertion/crash, ensuring it is a recorded failure
    # rather than bringing down the report or any live experiment process.
    code='import ctypes,sys;lib=ctypes.CDLL(sys.argv[1]);sys.exit(lib.local_state_regression())'
    result=subprocess.run(['python3','-c',code,str(library)],capture_output=True,text=True)
    report=dict(passed=result.returncode==0,returncode=result.returncode,
        checks=['empty warm feedback after new reference and rejected binding',
            'empty warm feedback after reset','actual actuator feedback retained',
            'nonempty warm feedback semantics unchanged','retired static side cleared',
            'mixed reverse-forward reference connector uses current segment gear',
            'static feasible nominal outside common comfort eligible to rejoin',
            'checked native rejoin owns zero-progress turn instead of conflicting legacy recovery'],
        bounds_assertions=True,scope='controller state/math regression, not online safety or hardware validation',
        source_sha256={name:hashlib.sha256((build/'source'/name).read_bytes()).hexdigest()
            for name in ('online_api.cpp','online_kernel.hpp','mppi_v12.hpp','shared_collision.hpp')},
        stdout=result.stdout,stderr=result.stderr)
    (out/'tests.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2));assert report['passed'],report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--build',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();test(a.build,a.out)
