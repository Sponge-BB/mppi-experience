"""Rebuild only the changed worker; preserve verified installed map objects."""
import hashlib
import json
import os
from pathlib import Path
import resource
import shlex
import signal
import subprocess
import time

HERE = Path(__file__).resolve().parent
ROOT = Path('/home/l/remani_ws')
BUILD = ROOT/'build/mm_remani_planner/plan_env'
OLD = HERE.parent/'map_compute_20261110'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
baseline = json.loads((HERE/'baseline_object_verification.json').read_text())
assert sha(ROOT/'devel/lib/libplan_env.so') == baseline['installed_library_sha256']
flags = {}
for line in (BUILD/'CMakeFiles/plan_env.dir/flags.make').read_text().splitlines():
    if line.startswith('CXX_'):
        key, value = line.split('=', 1)
        append = key.endswith('+')
        key = key.rstrip('+ ').strip()
        flags[key] = (flags.get(key, '') if append else '')+' '+value.strip()

minimum_available = float('inf')
def restrict():
    os.nice(15)
    resource.setrlimit(resource.RLIMIT_AS, (2*1024**3, 2*1024**3))

def run(command, log):
    global minimum_available
    child = subprocess.Popen(command, cwd=str(BUILD), stdout=log,
        stderr=subprocess.STDOUT, preexec_fn=restrict, start_new_session=True)
    while child.poll() is None:
        available = next(int(s.split()[1]) for s in Path('/proc/meminfo').read_text().splitlines()
                         if s.startswith('MemAvailable:'))
        minimum_available = min(minimum_available, available)
        if available < 850*1024:
            os.killpg(child.pid, signal.SIGTERM)
            child.wait()
            raise RuntimeError('Stopped only this compiler: available RAM below 850 MiB')
        time.sleep(.25)
    if child.returncode:
        raise RuntimeError('Build failed; see map_worker_build.log')

source = ROOT/'src/mm_remani_planner/plan_env/src/esdf_worker.cpp'
source_sha = sha(source)
obj = HERE/'esdf_worker.cpp.o'
command = ['/usr/bin/c++']
for key in ('CXX_DEFINES', 'CXX_INCLUDES', 'CXX_FLAGS'):
    command.extend(shlex.split(flags.get(key, '')))
command.extend(['-o', str(obj), '-c', str(source)])
started = time.monotonic()
with (HERE/'map_worker_build.log').open('w') as log:
    run(command, log)
    link = shlex.split((BUILD/'CMakeFiles/plan_env.dir/link.txt').read_text())
    link[link.index('-o')+1] = str(HERE/'libplan_env.so')
    replacement = {'CMakeFiles/plan_env.dir/src/grid_map.cpp.o': str(OLD/'library_grid_map.cpp.o'),
                   'CMakeFiles/plan_env.dir/src/esdf_worker.cpp.o': str(obj)}
    run([replacement.get(arg, arg) for arg in link], log)
assert sha(source) == source_sha, 'Worker source changed during compilation'
report = dict(source_sha256=source_sha, library_sha256=sha(HERE/'libplan_env.so'),
    baseline_sha256=baseline['installed_library_sha256'],
    elapsed_s=time.monotonic()-started, minimum_available_kib=minimum_available,
    installed=False, build='One worker translation unit; previous map object verified by exact baseline relink')
(HERE/'map_worker_build.json').write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps(report, indent=2))
