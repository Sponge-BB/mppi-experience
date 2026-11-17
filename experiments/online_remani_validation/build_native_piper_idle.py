#!/usr/bin/env python3
"""Build the exact committed Piper REMANI baseline for an idle-state comparison.

Only a fresh, experiment-local snapshot and standalone build description are
created. No fixed-arm transform, source patch, running ROS node, hardware node,
production build, production source, or user configuration is changed.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tarfile


HERE = Path(__file__).resolve().parent
WORKSPACE = HERE.parents[1]
REPO = WORKSPACE / 'src'
COMMIT = 'f138ec82dce5c58be2e7028548ed317efe541c4f'
OUT = HERE / 'builds/native_piper_idle_20261105_01'
PACKAGES = ('mm_remani_planner', 'mm_controller', 'mm_remani', 'Utils/mm_utils')
PRODUCTS = ('remani_planner_node', 'mm_controller_node')


CMAKE = r'''cmake_minimum_required(VERSION 3.10)
project(native_piper_idle_baseline)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG")
find_package(catkin REQUIRED COMPONENTS roscpp roslib std_msgs sensor_msgs
  nav_msgs geometry_msgs control_msgs visualization_msgs quadrotor_msgs
  cv_bridge tf2_ros pcl_conversions)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ompl REQUIRED)
set(P ${CMAKE_CURRENT_SOURCE_DIR}/source/mm_remani_planner)
foreach(part plan_manage plan_env mm_config path_searching traj_opt traj_utils)
  include_directories(${P}/${part}/include)
endforeach()
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/source/mm_controller/include
  ${CMAKE_CURRENT_SOURCE_DIR}/source/Utils/mm_utils/include)
include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${OMPL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_library(native_env STATIC ${P}/plan_env/src/grid_map.cpp
  ${P}/plan_env/src/raycast.cpp)
target_link_libraries(native_env ${catkin_LIBRARIES} ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES})
add_library(native_model STATIC ${P}/mm_config/src/mm_config.cpp)
target_link_libraries(native_model native_env)
add_library(native_search STATIC ${P}/path_searching/src/kino_astar.cpp
  ${P}/path_searching/src/sample_mani_RRT.cpp ${P}/path_searching/src/rrt.cpp)
target_link_libraries(native_search native_model ${OMPL_LIBRARIES})
add_library(native_opt STATIC ${P}/traj_opt/src/poly_traj_optimizer.cpp)
target_link_libraries(native_opt native_search)
add_executable(remani_planner_node ${P}/plan_manage/src/remani_planner_node.cpp
  ${P}/plan_manage/src/remani_replan_fsm.cpp
  ${P}/plan_manage/src/planner_manager.cpp
  ${P}/plan_manage/src/planning_visualization.cpp)
target_link_libraries(remani_planner_node native_opt)
add_executable(mm_controller_node
  ${CMAKE_CURRENT_SOURCE_DIR}/source/mm_controller/src/mm_controller_node.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/source/mm_controller/src/mm_controller_fsm.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/source/mm_controller/src/controller_utils.cpp)
target_link_libraries(mm_controller_node ${catkin_LIBRARIES})
'''


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def git(*args):
    return subprocess.check_output(['git', '-C', str(REPO), *args])


def write_manifest(data):
    temporary = OUT / 'build_manifest.json.tmp'
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n')
    temporary.replace(OUT / 'build_manifest.json')


def inventory():
    return {str(p.relative_to(OUT / 'source')): sha(p)
            for p in sorted((OUT / 'source').rglob('*')) if p.is_file()}


def verify_blob_files():
    """Verify every archived file against the exact selected Git blob."""
    blobs = {}
    raw = git('ls-tree', '-rz', COMMIT, '--', *PACKAGES)
    for record in raw.split(b'\0'):
        if not record:
            continue
        metadata, name = record.split(b'\t', 1)
        mode, kind, oid = metadata.decode().split()
        relative = name.decode()
        if kind != 'blob' or mode not in ('100644', '100755'):
            raise RuntimeError('Unexpected archived object: ' + relative)
        path = OUT / 'source' / relative
        actual = subprocess.check_output(['git', 'hash-object', str(path)]).decode().strip()
        if actual != oid:
            raise RuntimeError('Snapshot differs from committed blob: ' + relative)
        blobs[relative] = dict(git_blob_sha1=oid, sha256=sha(path), mode=mode)
    if set(blobs) != set(inventory()):
        raise RuntimeError('Archived source inventory differs from Git tree')
    return blobs


def prepare():
    OUT.mkdir(parents=True, exist_ok=False)
    source = OUT / 'source'
    source.mkdir()
    archive = OUT / 'committed_source.tar'
    with archive.open('wb') as stream:
        subprocess.run(['git', '-C', str(REPO), 'archive', '--format=tar',
                        COMMIT, *PACKAGES], stdout=stream, check=True)
    with tarfile.open(str(archive)) as tar:
        for member in tar.getmembers():
            path = Path(member.name)
            if path.is_absolute() or '..' in path.parts or member.issym() or member.islnk():
                raise RuntimeError('Unexpected archive path or link: ' + member.name)
        tar.extractall(str(source))
    blobs = verify_blob_files()
    (OUT / 'CMakeLists.txt').write_text(CMAKE)
    (OUT / 'committed_blob_inventory.json').write_text(
        json.dumps(blobs, indent=2) + '\n')
    metadata = git('show', '-s', '--format=%H%n%ci%n%s', COMMIT).decode().splitlines()
    manifest = dict(schema=1, created=datetime.now(timezone.utc).isoformat(),
        build_complete=False, source_git_commit=COMMIT,
        source_git_ref='baseline/static-planning-20260721',
        source_git_commit_date=metadata[1], source_git_commit_subject=metadata[2],
        baseline_label='Original REMANI Piper-adapted static-planning baseline',
        not_unmodified_official_upstream=True,
        source_repository=str(REPO), source_packages=list(PACKAGES),
        source_root=str(source), source_blob_inventory='committed_blob_inventory.json',
        isolated_source_sha256=inventory(), source_archive_sha256=sha(archive),
        source_patches=[], source_algorithm_changes=[], arm_coordinates_frozen=False,
        production_modified=False, production_default=False,
        build_adapter='Standalone CMake only; original source files are byte-identical Git blobs',
        compiler_flags='C++17 -O2 -DNDEBUG; serial build',
        cmake_sha256=sha(OUT / 'CMakeLists.txt'), recipe_sha256=sha(__file__),
        external_dependencies='Installed ROS Noetic, Eigen, PCL, OpenCV, OMPL and existing generated ROS message headers',
        scope='Actual original Piper planner/controller; idle/no-goal comparison; free simulated base and joints',
        commands=[])
    write_manifest(manifest)
    print(json.dumps(dict(prepared=str(OUT), committed_files=len(blobs),
                          source_patches=[])), flush=True)
    return manifest


def build():
    manifest = json.loads((OUT / 'build_manifest.json').read_text())
    verify_blob_files()
    if manifest['isolated_source_sha256'] != inventory():
        raise RuntimeError('Source snapshot drift')
    if sha(OUT / 'CMakeLists.txt') != manifest['cmake_sha256']:
        raise RuntimeError('CMake adapter drift')
    env = dict(os.environ)
    env['PYTHONNOUSERSITE'] = '1'
    env['CMAKE_PREFIX_PATH'] = str(WORKSPACE / 'devel') + ':/opt/ros/noetic'
    commands = [
        ['cmake', '-S', str(OUT), '-B', str(OUT / 'build'),
         '-DCMAKE_BUILD_TYPE=Release', '-DPYTHON_EXECUTABLE=/usr/bin/python3'],
        ['cmake', '--build', str(OUT / 'build'), '--parallel', '1'],
    ]
    with (OUT / 'build.log').open('a', buffering=1) as log:
        for command in commands:
            record = dict(argv=command, started=datetime.now(timezone.utc).isoformat())
            manifest['commands'].append(record)
            write_manifest(manifest)
            log.write(json.dumps(record) + '\n')
            result = subprocess.run(command, env=env, stdout=log, stderr=subprocess.STDOUT)
            record.update(returncode=result.returncode,
                          ended=datetime.now(timezone.utc).isoformat())
            write_manifest(manifest)
            if result.returncode:
                raise RuntimeError('Build failed; retained exact command and build.log')
    verify_blob_files()
    manifest.update(build_complete=True,
                    native_binaries_sha256={n: sha(OUT / 'build' / n) for n in PRODUCTS},
                    build_log_sha256=sha(OUT / 'build.log'))
    write_manifest(manifest)
    print(json.dumps(dict(build_complete=True, build=str(OUT),
                          native_binaries_sha256=manifest['native_binaries_sha256'])), flush=True)
    return manifest


def check():
    manifest = json.loads((OUT / 'build_manifest.json').read_text())
    if not manifest['build_complete'] or manifest['source_patches']:
        raise RuntimeError('Completed unpatched baseline required')
    verify_blob_files()
    if manifest['isolated_source_sha256'] != inventory():
        raise RuntimeError('Source snapshot drift')
    if sha(OUT / 'CMakeLists.txt') != manifest['cmake_sha256']:
        raise RuntimeError('CMake adapter drift')
    for name, digest in manifest['native_binaries_sha256'].items():
        if sha(OUT / 'build' / name) != digest:
            raise RuntimeError('Native binary drift: ' + name)
    print(json.dumps(dict(verified=True, commit=COMMIT, source_patches=[],
                          binaries=list(PRODUCTS))), flush=True)
    return manifest


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('all', 'prepare', 'build', 'check'), default='all', nargs='?')
    args = parser.parse_args()
    if args.action in ('all', 'prepare'):
        prepare()
    if args.action in ('all', 'build'):
        build()
    if args.action == 'check':
        check()
