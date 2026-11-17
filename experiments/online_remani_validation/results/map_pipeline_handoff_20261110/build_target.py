import os,resource,shlex,subprocess,sys,signal,time
from pathlib import Path
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
BUILD=ROOT/'build/mm_remani_planner/plan_env'
kind=sys.argv[1]
target,source=('plan_env','src/grid_map.cpp') if kind=='library' else ('test_grid_map_parallel','test/test_grid_map_parallel.cpp')
flags={}
for line in (BUILD/('CMakeFiles/'+target+'.dir/flags.make')).read_text().splitlines():
 if line.startswith('CXX_'):
  name,value=line.split('=',1);append=name.endswith('+');name=name.rstrip('+ ').strip()
  flags[name]=(flags.get(name,'') if append else '')+' '+value.strip()
obj=HERE/('grid_map.cpp.o' if kind=='library' else 'test_grid_map_parallel.cpp.o')
cmd=['/usr/bin/c++']
for name in ['CXX_DEFINES','CXX_INCLUDES','CXX_FLAGS']:cmd+=shlex.split(flags.get(name,''))
if '-g0' not in cmd:cmd+=['-g0','--param=ggc-min-expand=5','--param=ggc-min-heapsize=16384']
cmd+=['-o',str(obj),'-c',str(ROOT/'src/mm_remani_planner/plan_env'/source)]
def restrict():
 os.nice(15)
 resource.setrlimit(resource.RLIMIT_AS,(6*1024**3,6*1024**3))
with (HERE/(kind+'_build.log')).open('w') as log:
 print('Compiling one '+kind+' object, low priority, stop only compiler if available RAM < 650 MiB',flush=True)
 child=subprocess.Popen(cmd,cwd=str(BUILD),stdout=log,stderr=subprocess.STDOUT,preexec_fn=restrict,start_new_session=True)
 while child.poll() is None:
  available=next(int(line.split()[1]) for line in Path('/proc/meminfo').read_text().splitlines() if line.startswith('MemAvailable:'))
  if available<650*1024:
   os.killpg(child.pid,signal.SIGTERM);child.wait();raise RuntimeError('Stopped only compiler to preserve host memory')
  time.sleep(1)
 if child.returncode:raise RuntimeError('Compilation failed; see '+str(HERE/(kind+'_build.log')))
 link=shlex.split((BUILD/('CMakeFiles/'+target+'.dir/link.txt')).read_text())
 link[link.index('-o')+1]=str(HERE/('libplan_env.so' if kind=='library' else 'test_grid_map_parallel'))
 for i,s in enumerate(link):
  if s==('CMakeFiles/'+target+'.dir/'+source+'.o'):link[i]=str(obj)
  if kind!='library' and s==str(ROOT/'devel/lib/libplan_env.so'):link[i]=str(HERE/'libplan_env.so')
 subprocess.run(link,cwd=str(BUILD),stdout=log,stderr=subprocess.STDOUT,preexec_fn=restrict,check=True)
 print(kind+' compile/link passed; production library not replaced yet',flush=True)
