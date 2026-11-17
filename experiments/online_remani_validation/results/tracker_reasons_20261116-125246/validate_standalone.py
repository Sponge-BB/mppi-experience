#!/usr/bin/env python3
"""Standalone Eigen/gtest only. Never builds a ROS/PCL node or writes devel."""
import json, os, signal, subprocess, time
from pathlib import Path
STAGE = Path(__file__).resolve().parent
BUILD = STAGE / 'standalone_build'
BUILD.mkdir(exist_ok=True)
FLAGS = ['/usr/bin/c++', '-std=c++14', '-O0', '-g0', '--param=ggc-min-expand=5', '--param=ggc-min-heapsize=16384', '-I' + str(STAGE/'src/mm_remani/include'), '-I/usr/include/eigen3']
commands = [
 ('core', FLAGS + ['-c', str(STAGE/'src/mm_remani/src/dynamic_obstacle_tracker_core.cpp'), '-o', str(BUILD/'core.o')]),
 ('test', FLAGS + ['-c', str(STAGE/'src/mm_remani/test/test_dynamic_obstacle_tracker_core.cpp'), '-o', str(BUILD/'test.o')]),
 ('link', ['/usr/bin/c++', str(BUILD/'test.o'), str(BUILD/'core.o'), '/usr/lib/aarch64-linux-gnu/libgtest_main.a', '/usr/lib/aarch64-linux-gnu/libgtest.a', '-pthread', '-o', str(BUILD/'test_tracker_diag')]),
 ('run', [str(BUILD/'test_tracker_diag'), '--gtest_color=no', '--gtest_output=xml:'+str(BUILD/'tests.xml')])]
def memory():
 proc=[]
 for stat in Path('/proc').glob('[0-9]*/stat'):
  try:
   content=stat.read_text(); end=content.rindex(')'); fields=content[end+2:].split()
   proc.append((int(stat.parent.name),int(fields[2]),int(fields[21])*os.sysconf('SC_PAGE_SIZE')))
  except (FileNotFoundError,ProcessLookupError,PermissionError,ValueError,IndexError): pass
 return proc
results=[]
for name,command in commands:
 started=time.monotonic(); peak=0; guard=None
 with (BUILD/(name+'.log')).open('w') as log:
  proc=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
  while proc.poll() is None:
   rows=memory(); used=sum(rss for pid,pgrp,rss in rows if pgrp==proc.pid);peak=max(peak,used)
   available=next(int(line.split()[1])*1024 for line in Path('/proc/meminfo').read_text().splitlines() if line.startswith('MemAvailable:'))
   if used>512*1024**2 or available<1536*1024**2:
    guard='process_group_over_512MiB' if used>512*1024**2 else 'system_available_under_1536MiB'
    os.killpg(proc.pid,signal.SIGTERM);break
   time.sleep(.10)
  try: code=proc.wait(timeout=5)
  except subprocess.TimeoutExpired:
   os.killpg(proc.pid,signal.SIGKILL);code=proc.wait()
 result={'name':name,'command':command,'returncode':code,'guard':guard,'peak_process_group_rss_MiB':round(peak/1024**2,2),'elapsed_s':round(time.monotonic()-started,3),'log':str(BUILD/(name+'.log'))}
 results.append(result);print(json.dumps(result),flush=True)
 (STAGE/'standalone_validation.json').write_text(json.dumps({'status':'in_progress' if code==0 else 'failed','results':results},indent=2)+'\n')
 if code or guard: raise SystemExit(code or 1)
(STAGE/'standalone_validation.json').write_text(json.dumps({'status':'passed','scope':'temporary staged core and test only; node not compiled; no ROS or hardware','results':results},indent=2)+'\n')
