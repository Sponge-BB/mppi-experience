#!/usr/bin/python3
import os, resource, pathlib, subprocess, shlex, json, time
os.nice(15);resource.setrlimit(resource.RLIMIT_AS,(1024**3,1024**3))
r=pathlib.Path(__file__).resolve().parent;b=r/'build';s=r/'src/mm_remani'; records=[]
base=['g++','--param','ggc-min-expand=5','--param','ggc-min-heapsize=16384','-std=c++14','-O2','-g0','-pthread','-I'+str(s/'include'),'-I/usr/include/eigen3']
def run(name,cmd):
 t=time.monotonic()
 with (b/(name+'.log')).open('w') as f:p=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT)
 records.append(dict(name=name,command=cmd,returncode=p.returncode,elapsed_s=time.monotonic()-t,peak_rss_kib=resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss));(b/'fixture_steps.json').write_text(json.dumps(records,indent=2)+'\n');print(name,p.returncode,flush=True)
 if p.returncode:raise SystemExit(p.returncode)
run('core_short_circuit',base+['-c',str(s/'src/dynamic_obstacle_tracker_core.cpp'),'-o',str(b/'core.o')])
run('fixture_core',base+['-c',str(r/'fixture_core.cpp'),'-o',str(b/'fixture_core.o')])
libs=shlex.split(subprocess.check_output(['pkg-config','--libs','roscpp','tf2_ros','pcl_common-1.10','pcl_filters-1.10','pcl_segmentation-1.10','pcl_conversions'],text=True))
run('node_link_short_circuit',['g++','-pthread',str(b/'node.o'),str(b/'core.o')]+libs+['-Wl,-rpath,/opt/ros/noetic/lib','-o',str(b/'dynamic_obstacle_tracker_node')])
run('fixture_link',['g++','-pthread',str(b/'node.o'),str(b/'fixture_core.o')]+libs+['-Wl,-rpath,/opt/ros/noetic/lib','-o',str(b/'dynamic_obstacle_tracker_fixture')])
run('gtest_diagnostic_toggle',base+[str(s/'test/test_dynamic_obstacle_tracker_core.cpp'),str(b/'core.o'),'-lgtest','-lgtest_main','-o',str(b/'test_dynamic_obstacle_tracker_core')])
run('gtest_diagnostic_toggle_run',[str(b/'test_dynamic_obstacle_tracker_core'),'--gtest_output=xml:'+str(b/'gtest.xml')])
