#!/usr/bin/env python3
"""Small serial experimental child: source-stamped optional fast refresh."""
import argparse
import copy
from datetime import datetime
import difflib
import json
import os
from pathlib import Path
import shutil

import build_active_arm as parent
import build_arm_box_fast_only as bounded
from build_feasibility_stop import source_inventory

HERE = Path(__file__).resolve().parent
BASE = parent.OUT
OUT = HERE / 'builds/arm_box_refresh_20261115_01'
UNITS = ('kino_astar.cpp', 'poly_traj_optimizer.cpp', 'remani_replan_fsm.cpp', 'planner_manager.cpp')
ARCHIVES = ('libexp_search.a', 'libexp_opt.a')
sha = bounded.sha


def inputs():
    return {n:sha(HERE/n) for n in ('build_arm_box_refresh.py', 'arm_box_refresh_transform.py',
                                   'arm_box_refresh_policy.hpp')}


def recipe():
    r = copy.deepcopy(parent.check()['active_arm_recipe'])
    def convert(words):
        return [w.replace(str(BASE/'source'),str(OUT/'source')).replace(str(BASE/'build'),str(OUT/'build')) for w in words]
    r['compile_commands'] = [row for row in r['compile_commands'] if Path(row['source']).name in UNITS]
    for row in r['compile_commands']:
        row['source'] = row['source'].replace(str(BASE),str(OUT))
        row['object'] = row['object'].replace(str(BASE),str(OUT))
        row['command'] = convert(row['command'])
        row['command'][1:1] = ['-g0','--param=ggc-min-expand=5','--param=ggc-min-heapsize=16384']
    r['archive_commands'] = [convert(c) for c in r['archive_commands'] if Path(c[2]).name in ARCHIVES]
    r['link_commands'] = {'remani_planner_node':convert(r['link_commands']['remani_planner_node'])}
    return r


def prepare():
    from arm_box_refresh_transform import transform_sources
    m = parent.check()
    if OUT.exists():raise FileExistsError(str(OUT))
    OUT.mkdir();shutil.copytree(BASE/'source',OUT/'source');(OUT/'build').mkdir()
    original = {p.name:p.read_text() for p in (BASE/'source').glob('*') if p.suffix in ('.cpp','.hpp')}
    changed = transform_sources(original)
    differences=[]
    for name,text in changed.items():
        if text==original[name]:continue
        (OUT/'source'/name).write_text(text)
        differences.extend(difflib.unified_diff(original[name].splitlines(True),text.splitlines(True),
                                               fromfile='parent/'+name,tofile='child/'+name))
    shutil.copy2(HERE/'arm_box_refresh_policy.hpp',OUT/'source/arm_box_refresh_policy.hpp')
    (OUT/'source_change.diff').write_text(''.join(differences))
    excluded={n+'.o' for n in UNITS}|set(ARCHIVES)|{'remani_planner_node'}
    reused={}
    for p in sorted((BASE/'build').iterdir()):
        if p.is_file() and p.name not in excluded:
            (OUT/'build'/p.name).symlink_to(p)
            reused[p.name]={'target':str(p),'sha256':sha(p)}
    manifest=dict(schema='arm_box_refresh_build_v1',parent_build=str(BASE),
        parent_manifest_sha256=sha(BASE/'build_manifest.json'),prepared_only=True,created=datetime.now().isoformat(),
        inputs_sha256=inputs(),source_sha256=source_inventory(OUT/'source'),recipe=recipe(),reused=reused,
        production_modified=False,hardware_started=False,
        scope='isolated numerical arm-box optional refresh experiment; original bounds and safety acceptance')
    (OUT/'build_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return check()


def check():
    parent.check()
    m=json.loads((OUT/'build_manifest.json').read_text())
    assert sha(BASE/'build_manifest.json')==m['parent_manifest_sha256']
    assert inputs()==m['inputs_sha256']
    assert source_inventory(OUT/'source')==m['source_sha256']
    assert recipe()==m['recipe']
    for n,v in m['reused'].items():
        p=OUT/'build'/n
        assert p.is_symlink() and os.readlink(p)==v['target'] and sha(p)==v['sha256'],n
    if not m['prepared_only']:
        for n,h in m['compiled_sha256'].items():assert sha(OUT/'build'/n)==h,n
    return m


def build():
    m=check() if OUT.exists() else prepare()
    if not m['prepared_only']:return m
    bounded.OUT=OUT
    resources=[]
    for row in m['recipe']['compile_commands']:
        name=Path(row['source']).name
        print('COMPILE',name,flush=True)
        resources.append(bounded.run_bounded('compile_'+name,row['command'],m['recipe']['cwd']))
    for cmd in m['recipe']['archive_commands']:
        resources.append(bounded.run_bounded('archive_'+Path(cmd[2]).stem,cmd,m['recipe']['cwd']))
    resources.append(bounded.run_bounded('link_planner',m['recipe']['link_commands']['remani_planner_node'],m['recipe']['cwd']))
    check()
    m.update(prepared_only=False,built=datetime.now().isoformat(),build_resources=resources,
             compiled_sha256={n:sha(OUT/'build'/n) for n in (*[u+'.o' for u in UNITS],*ARCHIVES,'remani_planner_node')})
    (OUT/'build_manifest.json').write_text(json.dumps(m,indent=2)+'\n')
    return check()


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('action',choices=['prepare','build','check'])
    a=p.parse_args();m={'prepare':prepare,'build':build,'check':check}[a.action]()
    print(json.dumps(dict(path=str(OUT),prepared_only=m['prepared_only'])),flush=True)
