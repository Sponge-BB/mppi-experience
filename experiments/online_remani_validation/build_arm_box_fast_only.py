#!/usr/bin/env python3
"""Build an isolated first-reference-only child of the active-arm ROS planner.

The initial native planning transaction is unchanged.  Once it reaches
EXEC_TRAJ, its execution-state authorization remains published, while the
three FSM timer entry points no longer request another slow trajectory.
Mapping, ESDF callbacks and the native 3-D collision export are untouched.
This experimental binary requires local_execution_owner=true; the fast
executor still owns its original rollout, braking and input-freshness checks.
No production source, parent artifact or ROS process is changed here.
"""
import argparse
import copy
from datetime import datetime, timezone
import difflib
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import build_active_arm as parent_builder
from build_feasibility_stop import source_inventory


HERE = Path(__file__).resolve().parent
BASE = parent_builder.OUT
OUT = HERE / 'builds/arm_box_fast_only_20261114_01'
SOURCE = 'remani_replan_fsm.cpp'
OBJECT = SOURCE + '.o'
MAX_GROUP_RSS = int(1.8 * 1024**3)
MIN_AVAILABLE = int(1.5 * 1024**3)


def sha(path):
    value = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            value.update(block)
    return value.hexdigest()


def json_sha(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def write_manifest(manifest):
    temporary = OUT / 'build_manifest.json.tmp'
    temporary.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
    temporary.replace(OUT / 'build_manifest.json')


def transform(source):
    anchor = 'namespace remani_planner\n{\n'
    if source.count(anchor) != 1:
        raise ValueError('Unexpected planner namespace')
    source = source.replace(anchor, anchor + '''// Experiment-only latch: never changes class layout or mapping callbacks.
namespace arm_box_fast_only {
inline bool& frozen(){static bool value=false;return value;}
inline bool hold(bool local_owner,bool executing,int trajectory_id){
  if(!local_owner)return false;
  if(!frozen()&&executing&&trajectory_id>0){
    frozen()=true;
    ROS_WARN("[ArmBoxFastOnly] Initial native reference is frozen; subsequent slow planning is disabled. Map/ESDF and fast collision checks remain active.");
    dual_probe::emit("arm_box_slow_layer_frozen",dual_probe::Fields()
      .add("guide_id",trajectory_id).add("initial_reference_retained","true")
      .add("mapping_callbacks_unchanged","true"));
  }
  return frozen();
}
} // namespace arm_box_fast_only

''', 1)
    cooperative = '  const ros::Time now=ros::Time::now();publishExecutionState();\n'
    if source.count(cooperative) != 1:
        raise ValueError('Unexpected cooperative callback entry')
    source = source.replace(cooperative, cooperative + '''  if(arm_box_fast_only::hold(local_execution_owner_,exec_state_==EXEC_TRAJ,
      planner_manager_->traj_container_.singul_traj_data.traj_id))return;
''', 1)
    execute = '''  void REMANIReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {
    applyLatestOdom();
    exec_timer_.stop(); // To avoid blockage
'''
    if source.count(execute) != 1:
        raise ValueError('Unexpected execution callback entry')
    source = source.replace(execute, '''  void REMANIReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {
    applyLatestOdom();
    // Return before stopping the timer.  Keep the first latched task authority
    // and reference alive even after their nominal polynomial time has ended.
    if(arm_box_fast_only::hold(local_execution_owner_,exec_state_==EXEC_TRAJ,
        planner_manager_->traj_container_.singul_traj_data.traj_id)){
      publishExecutionState();return;
    }
    exec_timer_.stop(); // To avoid blockage
''', 1)
    collision = '''  void REMANIReplanFSM::checkCollisionCallback(const ros::TimerEvent &e){
    applyLatestOdom();
'''
    if source.count(collision) != 1:
        raise ValueError('Unexpected collision callback entry')
    source = source.replace(collision, collision + '''    if(arm_box_fast_only::hold(local_execution_owner_,exec_state_==EXEC_TRAJ,
        planner_manager_->traj_container_.singul_traj_data.traj_id)){
      publishExecutionState();return;
    }
''', 1)
    return source


def parent_check():
    manifest = parent_builder.check()
    if manifest.get('prepared_only'):
        raise ValueError('Parent active-arm build is incomplete')
    return manifest


def make_recipe(parent):
    inherited = parent['active_arm_recipe']
    rows = [r for r in inherited['compile_commands'] if Path(r['source']).name == SOURCE]
    if len(rows) != 1:
        raise ValueError('Expected one parent FSM compile recipe')
    row = copy.deepcopy(rows[0])
    command = row['command']
    old_source, old_object = row['source'], row['object']
    row.update(source=str(OUT / 'source' / SOURCE), object=str(OUT / 'build' / OBJECT))
    command = [row['source'] if value == old_source else row['object'] if value == old_object else value
               for value in command]
    command[1:1] = ['-g0', '--param=ggc-min-expand=5', '--param=ggc-min-heapsize=16384']
    row['command'] = command
    original_link = inherited['link_commands']['remani_planner_node']
    prefix = str(BASE / 'build') + '/'
    link = [str(OUT / 'build' / Path(value).name) if value.startswith(prefix) else value
            for value in original_link]
    return dict(compile_commands=[row], link_commands={'remani_planner_node': link},
                cwd=inherited['cwd'], parent_recipe_sha256=json_sha(inherited),
                parent_fsm_recipe_sha256=json_sha(rows[0]),
                parent_link_recipe_sha256=json_sha(original_link),
                compiler_profile='unchanged -O2 C++17; no debug; bounded GCC garbage collection')


def prepare():
    parent = parent_check()
    if OUT.exists():
        raise FileExistsError('Refusing to overwrite experiment child: ' + str(OUT))
    (OUT / 'source').mkdir(parents=True)
    (OUT / 'build').mkdir()
    original = (BASE / 'source' / SOURCE).read_text()
    modified = transform(original)
    (OUT / 'source' / SOURCE).write_text(modified)
    (OUT / 'source_change.diff').write_text(''.join(difflib.unified_diff(
        original.splitlines(True), modified.splitlines(True), fromfile=str(BASE / 'source' / SOURCE),
        tofile=str(OUT / 'source' / SOURCE))))
    reused = {}
    for path in sorted((BASE / 'build').iterdir()):
        if path.is_file() and path.name not in (OBJECT, 'remani_planner_node'):
            (OUT / 'build' / path.name).symlink_to(path)
            reused[path.name] = dict(sha256=sha(path), target=str(path), mode=oct(path.stat().st_mode & 0o7777))
    recipe = make_recipe(parent)
    manifest = copy.deepcopy(parent)
    manifest.update(schema=1, created=datetime.now(timezone.utc).isoformat(), prepared_only=True,
        arm_box_fast_only_experiment=True, parent_build=str(BASE), source_root=str(OUT / 'source'),
        parent_manifest_sha256=sha(BASE / 'build_manifest.json'),
        parent_source_sha256=parent['isolated_source_sha256'],
        isolated_source_sha256=source_inventory(OUT / 'source'),
        parent_fsm_source_sha256=sha(BASE / 'source' / SOURCE),
        modified_fsm_source_sha256=sha(OUT / 'source' / SOURCE),
        arm_box_fast_only_builder_sha256=sha(__file__), arm_box_fast_only_recipe=recipe,
        arm_box_fast_only_recipe_sha256=json_sha(recipe), reused_artifacts=reused,
        source_change_sha256=sha(OUT / 'source_change.diff'),
        native_binaries_sha256={name:sha(OUT / 'build' / name)
            for name in ('mm_controller_node', 'dynamic_obstacle_tracker_node')},
        production_modified=False, hardware_started=False,
        requirement='local_execution_owner=true; one complete initial goal reference; no second goal',
        freeze_contract='latch at first EXEC_TRAJ with traj_id>0; retain task state/reference; return at all three FSM timer entries',
        completion_owner='numerical plant, not the frozen FSM; fast local executor remains independently safety checked',
        runtime_evidence='[ArmBoxFastOnly] log and arm_box_slow_layer_frozen dual timeline event',
        mapping_changes=[], controller_changes=[], scope='isolated numerical fast-only ablation, no hardware')
    write_manifest(manifest)
    return check()


def check():
    parent = parent_check()
    manifest = json.loads((OUT / 'build_manifest.json').read_text())
    if sha(BASE / 'build_manifest.json') != manifest['parent_manifest_sha256']:
        raise ValueError('Parent manifest changed')
    if parent['isolated_source_sha256'] != manifest['parent_source_sha256']:
        raise ValueError('Parent source inventory changed')
    for path, expected in ((BASE / 'source' / SOURCE, manifest['parent_fsm_source_sha256']),
                           (OUT / 'source' / SOURCE, manifest['modified_fsm_source_sha256']),
                           (Path(__file__), manifest['arm_box_fast_only_builder_sha256']),
                           (OUT / 'source_change.diff', manifest['source_change_sha256'])):
        if sha(path) != expected:
            raise ValueError('Source/evidence hash mismatch: ' + str(path))
    if source_inventory(OUT / 'source') != manifest['isolated_source_sha256']:
        raise ValueError('Child source inventory changed')
    recipe = make_recipe(parent)
    if recipe != manifest['arm_box_fast_only_recipe'] or json_sha(recipe) != manifest['arm_box_fast_only_recipe_sha256']:
        raise ValueError('Child recipe changed')
    for name, saved in manifest['reused_artifacts'].items():
        path = OUT / 'build' / name
        if not path.is_symlink() or os.readlink(path) != saved['target'] or sha(path) != saved['sha256']:
            raise ValueError('Reused artifact mismatch: ' + name)
        if oct(path.stat().st_mode & 0o7777) != saved['mode']:
            raise ValueError('Reused artifact mode changed: ' + name)
    if not manifest['prepared_only']:
        for name, expected in manifest['compiled_sha256'].items():
            if sha(OUT / 'build' / name) != expected:
                raise ValueError('Compiled artifact changed: ' + name)
        for name, expected in manifest['native_binaries_sha256'].items():
            if sha(OUT / 'build' / name) != expected:
                raise ValueError('Native binary changed: ' + name)
    return manifest


def available_bytes():
    for line in Path('/proc/meminfo').read_text().splitlines():
        if line.startswith('MemAvailable:'):
            return int(line.split()[1]) * 1024
    raise RuntimeError('MemAvailable is unavailable')


def group_rss_bytes(group):
    pages = 0
    for path in Path('/proc').iterdir():
        if not path.name.isdigit():
            continue
        try:
            text = (path / 'stat').read_text()
            fields = text[text.rfind(')') + 2:].split()
            if int(fields[2]) == group:
                pages += int(fields[21])
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            pass
    return pages * os.sysconf('SC_PAGE_SIZE')


def stop_owned_group(process):
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGTERM)
    try:
        process.wait(timeout=3.)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=5.)


def run_bounded(label, command, cwd):
    start = time.monotonic()
    available = available_bytes()
    if available < MIN_AVAILABLE:
        raise RuntimeError('Insufficient memory before owned build: %.1f MiB available' % (available / 1024**2))
    stats = dict(command=command, cwd=cwd, serial=True, peak_group_rss_bytes=0,
                 minimum_available_bytes=available, abort_reason=None)
    path = OUT / (label + '_resources.json')
    process = None
    try:
        with (OUT / (label + '.log')).open('w') as log:
            process = subprocess.Popen(command, cwd=cwd, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True)
            while process.poll() is None:
                rss, available = group_rss_bytes(process.pid), available_bytes()
                stats['peak_group_rss_bytes'] = max(stats['peak_group_rss_bytes'], rss)
                stats['minimum_available_bytes'] = min(stats['minimum_available_bytes'], available)
                if rss > MAX_GROUP_RSS or available < MIN_AVAILABLE:
                    stats['abort_reason'] = 'owned_group_rss_limit' if rss > MAX_GROUP_RSS else 'available_memory_limit'
                    stop_owned_group(process)
                    raise RuntimeError(stats['abort_reason'])
                time.sleep(.2)
            if process.returncode:
                raise RuntimeError('%s failed with exit code %d; inspect %s' %
                                   (label, process.returncode, OUT / (label + '.log')))
    finally:
        if process is not None:
            stop_owned_group(process)
            stats['returncode'] = process.returncode
        stats['elapsed_s'] = time.monotonic() - start
        path.write_text(json.dumps(stats, indent=2) + '\n')
    return stats


def build():
    manifest = check() if OUT.exists() else prepare()
    if not manifest['prepared_only']:
        return manifest
    recipe = manifest['arm_box_fast_only_recipe']
    resources = []
    resources.append(run_bounded('compile_fsm', recipe['compile_commands'][0]['command'], recipe['cwd']))
    resources.append(run_bounded('link_planner', recipe['link_commands']['remani_planner_node'], recipe['cwd']))
    check()
    manifest.update(prepared_only=False, built=datetime.now(timezone.utc).isoformat(),
                    compiled_sha256={name:sha(OUT / 'build' / name) for name in (OBJECT, 'remani_planner_node')},
                    native_binaries_sha256={name:sha(OUT / 'build' / name) for name in
                        ('remani_planner_node', 'mm_controller_node', 'dynamic_obstacle_tracker_node')},
                    build_resources=resources)
    write_manifest(manifest)
    return check()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('prepare', 'build', 'check'))
    args = parser.parse_args()
    result = {'prepare':prepare, 'build':build, 'check':check}[args.action]()
    print(json.dumps(dict(build=str(OUT), action=args.action, prepared_only=result['prepared_only']), indent=2), flush=True)
