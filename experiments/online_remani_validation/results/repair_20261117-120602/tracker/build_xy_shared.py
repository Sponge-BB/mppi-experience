#!/usr/bin/python3
import pathlib,subprocess,os,resource,shlex,time,json,hashlib
r=pathlib.Path(__file__).resolve().parent;s=r/'behavior_candidate';b=s/'build';b.mkdir(exist_ok=True)
os.nice(15);resource.setrlimit(resource.RLIMIT_AS,(1024**3,1024**3));records=[]
base=['g++','--param','ggc-min-expand=5','--param','ggc-min-heapsize=16384','-std=c++14','-O2','-g0','-pthread','-I'+str(s/'include'),'-I/usr/include/eigen3']
def run(name,cmd):
 start=time.monotonic()
 with (b/(name+'.log')).open('w') as f:p=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT)
 records.append({'name':name,'command':cmd,'returncode':p.returncode,'elapsed_s':time.monotonic()-start,'peak_rss_kib':resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss});(b/'steps.json').write_text(json.dumps(records,indent=2)+'\n');print(name,p.returncode,flush=True)
 if p.returncode:raise SystemExit(p.returncode)
run('core_pic',base+['-fPIC','-c',str(s/'src/dynamic_obstacle_tracker_core.cpp'),'-o',str(b/'core_pic.o')])
run('core_shared',['g++','-shared',str(b/'core_pic.o'),'-Wl,-soname,libdynamic_obstacle_tracker_core.so','-o',str(b/'libdynamic_obstacle_tracker_core.so')])
libs=shlex.split(subprocess.check_output(['pkg-config','--libs','roscpp','tf2_ros','pcl_common-1.10','pcl_filters-1.10','pcl_segmentation-1.10','pcl_conversions'],text=True))
run('node_shared',['g++','-pthread',str(r/'build/node.o'),'-L'+str(b),'-ldynamic_obstacle_tracker_core']+libs+['-Wl,-rpath,/home/l/remani_ws/devel/lib:/opt/ros/noetic/lib','-o',str(b/'dynamic_obstacle_tracker_node')])
run('gtest_all',base+[str(s/'test/test_recorded_planar_surface.cpp'),'-lgtest','-lgtest_main','-o',str(b/'test_recorded_planar_surface')])
run('gtest_all_run',[str(b/'test_recorded_planar_surface'),'--gtest_output=xml:'+str(b/'gtest.xml')])
manifest={str(p.relative_to(r)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [s/'src/dynamic_obstacle_tracker_core.cpp',s/'src/dynamic_obstacle_tracker_node.cpp',s/'include/mm_remani/dynamic_obstacle_tracker_core.h',r/'build/node.o',b/'libdynamic_obstacle_tracker_core.so',b/'dynamic_obstacle_tracker_node']};(b/'shared_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
