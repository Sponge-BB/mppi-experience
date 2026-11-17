#!/usr/bin/env python3
"""Standalone Eigen/gtest only. Never builds a ROS/PCL node or writes devel."""
import json, os, signal, subprocess, time
from pathlib import Path
STAGE = Path(__file__).resolve().parent
BUILD = STAGE / 'standalone_build'
BUILD.mkdir(exist_ok=True)
FLAGS = ['/usr/bin/c++', '-std=c++14', '-O0', '-g0', '--param=ggc-min-expand=5', '--param=ggc-min-heapsize=16384', '-I' + str(STAGE/'src/mm_remani/include'), '-I/usr/include/eigen3']
commands = [
 ('base_core', FLAGS[:6] + ['-I'+str(STAGE/'original/src/mm_remani/include'), '-I/usr/include/eigen3', '-c', str(STAGE/'original/src/mm_remani/src/dynamic_obstacle_tracker_core.cpp'), '-o', str(BUILD/'base_core.o')]),
 ('compare_fixture', FLAGS[:6] + ['-I'+str(STAGE/'original/src/mm_remani/include'), '-I/usr/include/eigen3', '-c', str(STAGE/'compare_original_fixture.cpp'), '-o', str(BUILD/'fixture.o')]),
 ('base_link', ['/usr/bin/c++', str(BUILD/'fixture.o'), str(BUILD/'base_core.o'), '-o', str(BUILD/'baseline_fixture')]),
 ('candidate_link', ['/usr/bin/c++', str(BUILD/'fixture.o'), str(BUILD/'core.o'), '-o', str(BUILD/'candidate_fixture')]),
 ('baseline_output', [str(BUILD/'baseline_fixture')]),
 ('candidate_output', [str(BUILD/'candidate_fixture')])]

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
 (STAGE/'original_comparison_validation.json').write_text(json.dumps({'status':'in_progress' if code==0 else 'failed','results':results},indent=2)+'\n')
 if code or guard: raise SystemExit(code or 1)
(STAGE/'original_comparison_validation.json').write_text(json.dumps({'status':'passed','scope':'original vs candidate core; same driver compiled with original header; no ROS/PCL or hardware','results':results},indent=2)+'\n')

import hashlib
base=(BUILD/'baseline_output.log').read_bytes();candidate=(BUILD/'candidate_output.log').read_bytes()
comparison={'byte_identical':base==candidate,'bytes_per_output':len(base),'lines_per_output':len(base.splitlines()),'baseline_sha256':hashlib.sha256(base).hexdigest(),'candidate_sha256':hashlib.sha256(candidate).hexdigest()}
assert comparison['byte_identical'], 'old and staged core behavior changed'
result_path=STAGE/'original_comparison_validation.json'
result=json.loads(result_path.read_text());result['comparison']=comparison;result_path.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(comparison))
