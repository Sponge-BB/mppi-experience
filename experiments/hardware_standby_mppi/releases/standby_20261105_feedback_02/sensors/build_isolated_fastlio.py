#!/usr/bin/python3
"""Serial isolated FASTLIO build. This script never launches the executable."""
import hashlib
import json
import os
from pathlib import Path
import resource
import shlex
import shutil
import subprocess
import time

SENSORS = Path(__file__).resolve().parent
RELEASE = SENSORS.parent


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def bound_compiler_memory():
    # Limit each compiler subprocess, leaving the desktop and ROS observer room.
    ceiling = 2600*1024*1024
    resource.setrlimit(resource.RLIMIT_AS, (ceiling, ceiling))


def main():
    flags = {}
    for line in (SENSORS/'config/original_flags.make').read_text().splitlines():
        if ' = ' in line:
            key, value = line.split(' = ', 1); flags[key] = value
    frozen = SENSORS/'source/FAST_LIO'
    runtime = RELEASE/'run/fastlio_runtime'
    for directory in ('Log', 'PCD'):
        (runtime/directory).mkdir(parents=True, exist_ok=True)
    defines = shlex.split(flags['CXX_DEFINES'])
    defines = [('-DROOT_DIR="'+str(runtime)+'/"') if a.startswith('-DROOT_DIR=') else a for a in defines]
    includes = shlex.split(flags['CXX_INCLUDES'])
    replacements = {
        '-I/home/l/remani_ws/devel/include': '-I'+str(SENSORS/'source/generated_include'),
        '-I/home/l/ws_livox/devel/include': '-I'+str(SENSORS/'source/generated_include'),
        '-I/home/l/remani_ws/src/NEXTE_Sentry_Nav/sentry_slam/FAST_LIO/include': '-I'+str(frozen/'include')}
    includes = [replacements.get(a, a) for a in includes]
    if any('/home/l/remani_ws/src/' in a or '/devel/' in a for a in includes):
        raise RuntimeError('unfrozen include directory')
    common = ['/usr/bin/c++', '-std=c++14', '-O1', '-g0', '-pthread', '-fexceptions',
              '-fopenmp', '--param=ggc-min-expand=20', '--param=ggc-min-heapsize=32768']+defines+includes
    units = ['src/laserMapping.cpp', 'include/ikd-Tree/ikd_Tree.cpp', 'src/preprocess.cpp']
    objects = [SENSORS/'build'/(Path(u).stem+'.o') for u in units]
    commands = [common+['-c', str(frozen/u), '-o', str(o)] for u, o in zip(units, objects)]
    old_link = shlex.split((SENSORS/'config/original_link.txt').read_text())
    old_objects = ['CMakeFiles/fastlio_mapping.dir/'+u+'.o' for u in units]
    mapping = {old: str(new) for old, new in zip(old_objects, objects)}
    link = [mapping.get(a, a) for a in old_link if a not in ('-g', '-O3')]
    link[link.index('-o')+1] = str(SENSORS/'bin/feedback02_fastlio_mapping')
    commands.append(link)
    inventory = {str(p.relative_to(SENSORS)): sha(p) for p in (SENSORS/'source').rglob('*') if p.is_file()}
    manifest = dict(status='building', serial=True, compiler_address_space_limit_mib=2600,
        optimization='-O1 -g0', runtime_root=str(runtime), source_sha256=inventory,
        commands=commands, hardware_started=False)
    target = SENSORS/'build_manifest.json'
    target.write_text(json.dumps(manifest, indent=2)+'\n')
    with (SENSORS/'build/build.log').open('w', buffering=1) as stream:
        try:
            for index, command in enumerate(commands):
                print('ISOLATED_FASTLIO_STEP', index+1, 'OF', len(commands), flush=True)
                stream.write('COMMAND '+shlex.join(command)+'\n')
                begin = time.monotonic()
                subprocess.run(command, cwd=SENSORS/'build', stdout=stream, stderr=subprocess.STDOUT,
                    check=True, preexec_fn=bound_compiler_memory)
                manifest.setdefault('step_wall_s', []).append(time.monotonic()-begin)
                target.write_text(json.dumps(manifest, indent=2)+'\n')
        except BaseException as exc:
            manifest.update(status='failed', error=str(exc))
            target.write_text(json.dumps(manifest, indent=2)+'\n')
            raise
    binary = SENSORS/'bin/feedback02_fastlio_mapping'
    ldd = subprocess.run(['ldd', str(binary)], capture_output=True, text=True, check=True).stdout
    (SENSORS/'build/fastlio_ldd.txt').write_text(ldd)
    if 'not found' in ldd or '/devel/' in ldd:
        raise RuntimeError('unfrozen runtime dependency')
    manifest.update(status='complete', binary_sha256=sha(binary), dynamic_dependencies=ldd,
        old_environment_runtime_libraries=False)
    target.write_text(json.dumps(manifest, indent=2)+'\n')
    print('ISOLATED_FASTLIO_BUILD_COMPLETE', binary, flush=True)


if __name__ == '__main__':
    main()
