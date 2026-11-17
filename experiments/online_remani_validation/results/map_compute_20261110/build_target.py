"""Targeted serial build; never builds the rest of the workspace."""
import os
import resource
import shlex
import signal
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
BUILD = ROOT / 'build/mm_remani_planner/plan_env'
kind = sys.argv[1]
is_library = kind in ('library', 'library_worker')
target = 'plan_env' if is_library else (
    'test_esdf_worker' if kind == 'worker_test' else 'test_grid_map_parallel')
sources = ['src/grid_map.cpp', 'src/esdf_worker.cpp'] if kind == 'library' else (
    ['src/esdf_worker.cpp'] if kind == 'library_worker' else [
    'test/test_esdf_worker.cpp' if kind == 'worker_test' else 'test/test_grid_map_parallel.cpp'])
flags = {}
for line in (BUILD / ('CMakeFiles/' + target + '.dir/flags.make')).read_text().splitlines():
    if line.startswith('CXX_'):
        name, value = line.split('=', 1)
        append = name.endswith('+')
        name = name.rstrip('+ ').strip()
        flags[name] = (flags.get(name, '') if append else '') + ' ' + value.strip()

def restrict():
    os.nice(15)
    resource.setrlimit(resource.RLIMIT_AS, (6 * 1024**3, 6 * 1024**3))

def run(command, log):
    child = subprocess.Popen(command, cwd=str(BUILD), stdout=log,
        stderr=subprocess.STDOUT, preexec_fn=restrict, start_new_session=True)
    while child.poll() is None:
        available = next(int(line.split()[1]) for line in Path('/proc/meminfo').read_text().splitlines()
                         if line.startswith('MemAvailable:'))
        if available < 650 * 1024:
            os.killpg(child.pid, signal.SIGTERM)
            child.wait()
            raise RuntimeError('Stopped compiler only: available RAM < 650 MiB')
        time.sleep(1)
    if child.returncode:
        raise RuntimeError('Build failed; see ' + str(HERE / (kind + '_build.log')))

objects = {'CMakeFiles/plan_env.dir/src/grid_map.cpp.o': str(HERE / 'library_grid_map.cpp.o')} if kind == 'library_worker' else {}
with (HERE / (kind + '_build.log')).open('w') as log:
    for source in sources:
        obj = HERE / (kind + '_' + Path(source).name + '.o')
        command = ['/usr/bin/c++']
        if kind == 'before_test':
            command += ['-I' + str(HERE / 'before/include'), '-DMAP_BENCHMARK_OLD_ABI']
        for name in ['CXX_DEFINES', 'CXX_INCLUDES', 'CXX_FLAGS']:
            command += shlex.split(flags.get(name, ''))
        command += ['-g0', '--param=ggc-min-expand=5', '--param=ggc-min-heapsize=16384',
                    '-o', str(obj), '-c', str(ROOT / 'src/mm_remani_planner/plan_env' / source)]
        print('Serial low-priority compile: ' + source, flush=True)
        run(command, log)
        objects['CMakeFiles/' + target + '.dir/' + source + '.o'] = str(obj)
    link = shlex.split((BUILD / ('CMakeFiles/' + target + '.dir/link.txt')).read_text())
    link[link.index('-o') + 1] = str(HERE / ('libplan_env.so' if is_library else kind))
    for index, value in enumerate(link):
        if value in objects:
            link[index] = objects[value]
        if not is_library and value == str(ROOT / 'devel/lib/libplan_env.so'):
            link[index] = str(HERE / ('before/libplan_env.so' if kind == 'before_test' else 'libplan_env.so'))
    run(link, log)
    print(kind + ' compile/link passed; installed library unchanged', flush=True)
