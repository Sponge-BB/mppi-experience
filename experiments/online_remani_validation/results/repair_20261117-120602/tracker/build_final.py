#!/usr/bin/python3
import os, pathlib, resource, shlex, subprocess, time, json
root=pathlib.Path(__file__).resolve().parent
src=root/'src/mm_remani'; build=root/'build'; build.mkdir(exist_ok=True)
os.nice(15)
resource.setrlimit(resource.RLIMIT_AS,(int(1.2*1024**3),int(1.2*1024**3)))
base=['g++','--param','ggc-min-expand=5','--param','ggc-min-heapsize=16384','-std=c++14','-O2','-g0','-pthread','-I'+str(src/'include'),'-I/usr/include/eigen3']
packages=['roscpp','tf2_ros','pcl_common-1.10','pcl_filters-1.10','pcl_segmentation-1.10','pcl_conversions']
records=json.loads((build/'steps.json').read_text()) if (build/'steps.json').exists() else []
def run(name,cmd):
 start=time.monotonic()
 with (build/(name+'.log')).open('w') as log:
  r=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT)
 records.append(dict(name=name,returncode=r.returncode,elapsed_s=time.monotonic()-start,child_peak_rss_kib=resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss,command=cmd))
 (build/'steps.json').write_text(json.dumps(records,indent=2)+'\n')
 print(name,r.returncode,round(records[-1]['elapsed_s'],3),flush=True)
 if r.returncode:raise SystemExit(r.returncode)
 if records[-1]['child_peak_rss_kib'] > 1048576: raise SystemExit('RSS exceeded 1 GiB')
run('core',base+['-c',str(src/'src/dynamic_obstacle_tracker_core.cpp'),'-o',str(build/'core.o')])
run('gtest',base+[str(src/'test/test_dynamic_obstacle_tracker_core.cpp'),str(build/'core.o'),'-lgtest','-lgtest_main','-o',str(build/'test_dynamic_obstacle_tracker_core')])
run('gtest_run',[str(build/'test_dynamic_obstacle_tracker_core'),'--gtest_output=xml:'+str(build/'gtest.xml')])
cflags=shlex.split(subprocess.check_output(['pkg-config','--cflags']+packages,text=True))
libs=shlex.split(subprocess.check_output(['pkg-config','--libs']+packages,text=True))
run('node_compile',base+cflags+['-c',str(src/'src/dynamic_obstacle_tracker_node.cpp'),'-o',str(build/'node.o')])
run('node_link',['g++','-pthread',str(build/'node.o'),str(build/'core.o')]+libs+['-Wl,-rpath,/opt/ros/noetic/lib','-o',str(build/'dynamic_obstacle_tracker_node')])
