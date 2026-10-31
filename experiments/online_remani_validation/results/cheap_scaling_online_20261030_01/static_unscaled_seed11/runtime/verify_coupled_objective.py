#!/usr/bin/env python3
"""Reproducible offline gradient/contract checks; never starts a ROS graph."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

from build_native_v10 import compile_source

HERE=Path(__file__).resolve().parent


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--build',type=Path,required=True)
    p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();out=a.out.resolve();out.mkdir(parents=True,exist_ok=False)
    (out/'build').mkdir();m=json.loads((a.build/'build_manifest.json').read_text())
    parent=Path(m['parent_build']);results=[]
    names=['test_coupling_objective.cpp','test_coupling_collision.cpp',
           'test_coupling_native_geometry.cpp','test_coupling_native_prediction.cpp']
    for name in names:
        _,obj=compile_source(parent,out,'exp_opt',HERE/name,'unused',HERE)
        executable=out/'build'/Path(name).stem
        cmd=['/usr/bin/c++',obj,'-o',str(executable),'-L/opt/ros/noetic/lib',
             '-Wl,-rpath,/opt/ros/noetic/lib','-lroscpp','-lrostime',
             '-lrosconsole','-lroscpp_serialization','-pthread']
        subprocess.run(cmd,check=True)
        run=subprocess.run([str(executable)],capture_output=True,text=True)
        (out/(Path(name).stem+'.log')).write_text(run.stdout+run.stderr)
        print(run.stdout,flush=True)
        results.append(dict(test=name,returncode=run.returncode,
            source_sha256=hashlib.sha256((HERE/name).read_bytes()).hexdigest()))
        if run.returncode:raise RuntimeError('failed native check: '+name)
    run=subprocess.run([sys.executable,'-m','unittest','discover','-s',str(HERE),
                        '-p','test_report_coupled_objective.py','-v'],capture_output=True,text=True)
    (out/'report_contracts.log').write_text(run.stdout+run.stderr)
    if run.returncode:raise RuntimeError('report/runner contracts failed')
    results.append(dict(test='report/runner contracts',returncode=run.returncode))
    (out/'verification.json').write_text(json.dumps(dict(no_ros_graph=True,tests=results,
        all_passed=True,build_manifest_sha256=hashlib.sha256((a.build/'build_manifest.json').read_bytes()).hexdigest(),
        headers_sha256={str(path.name):hashlib.sha256(path.read_bytes()).hexdigest()
            for path in HERE.glob('coupling_*.hpp')}),indent=2)+'\n')
    print('OFFLINE_COUPLING_VERIFICATION_PASSED',out,flush=True)


if __name__=='__main__':main()
