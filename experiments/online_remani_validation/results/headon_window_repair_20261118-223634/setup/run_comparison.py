#!/usr/bin/python3
"""Isolated, serial replay of the existing three-way SIL with a moving box."""
import argparse,importlib.util,json,os,shutil,sys,xml.etree.ElementTree as ET
from pathlib import Path
CASE=Path(__file__).resolve().parents[1]
HERE=CASE.parents[1]
sys.path.insert(0,str(HERE))
import run_arm_box_navigation as runner
runner.CASE=CASE

def prepare(group):
    out=runner.prepare(group,45.,11)
    runtime=out/'runtime'
    shutil.copy2(runtime/'arm_box_scene.py',runtime/'arm_box_geometry.py')
    shutil.copy2(CASE/'setup/headon_scene.py',runtime/'arm_box_scene.py')
    sys.path.insert(0,str(runtime))
    sys.modules.pop('arm_box_scene',None)
    import arm_box_scene as scene
    sys.modules.pop('arm_box_scene',None)
    sys.path.pop(0)
    p=runtime/'arm_box_simulator.py';s=p.read_text()
    s=s.replace('box_gaps, obstacle_points, straight_path_precheck)', 'box_gaps, obstacle_points, straight_path_precheck, actor_at)')
    s=s.replace('self.box = None','self.box = None\n        self.box_runtime = None',1)
    needle='    def audit(self, elapsed, sim_time=None):\n'
    assert s.count(needle)==1
    s=s.replace(needle,needle+'        if self.box_runtime is not None:\n            self.box = actor_at(self.box_runtime, sim_time)\n',1)
    needle="            self.plant.box = runtime['box']"
    assert s.count(needle)==1
    s=s.replace(needle,"            self.plant.box_runtime = runtime\n"+needle,1)
    p.write_text(s)
    cfgpath=out/'active_arm_config.json';cfg=json.loads(cfgpath.read_text())
    cfg.update(scene_definition=scene.scene_definition(),goal=scene.scene_definition()['goal'],comparison='same approaching box, shared free base/arm plant; previous three-way SIL controller settings retained')
    cfgpath.write_text(json.dumps(cfg,ensure_ascii=False,indent=2)+'\n')
    (out/'scenario.json').write_text(json.dumps(scene.scene_definition(),ensure_ascii=False,indent=2)+'\n')
    from active_arm_model import PiperArmModel
    check=scene.straight_path_precheck(PiperArmModel.from_parameters(out/'parameters.yaml',out/'model/mm.urdf'))
    assert check['passed'],check
    (out/'geometry_precheck.json').write_text(json.dumps(check,indent=2)+'\n')
    tree=ET.parse(out/'experiment.launch');launch=tree.getroot()
    node=launch.find("node[@name='algorithm_simulator']/param[@name='scene']")
    node.set('value',scene.scene_definition()['scene'])
    # Store the exact executable/library bytes beside this run, not build cache.
    assets=out/'bin';assets.mkdir()
    package=out/'ros_pkgs/online_remani_experiment'
    for name in ('remani_planner_node','mm_controller_node','dynamic_obstacle_tracker_node'):
        link=package/name;shutil.copy2(link.resolve(),assets/name);link.unlink();link.symlink_to(assets/name)
    lib=launch.find("node[@name='active_arm_mppi']/param[@name='library']")
    if lib is not None:
        shutil.copy2(lib.get('value'),assets/'libonline_mppi.so');lib.set('value',str(assets/'libonline_mppi.so'))
    tree.write(out/'experiment.launch',encoding='unicode')
    manifest=json.loads((out/'manifest.json').read_text())
    manifest.update(new_box_experiment=False,approaching_box_experiment=True,production_modified=False,
        scenario=scene.scene_definition(),source_sha256={str(p.relative_to(out)):runner.sha(p) for p in runtime.glob('*.py')},
        runtime_sha256={str(p.relative_to(out)):runner.sha(p) for p in [out/'parameters.yaml',out/'experiment.launch',cfgpath,out/'scenario.json',out/'model/mm.urdf',*assets.iterdir()]})
    (out/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    return out

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('group',choices=['native','mppi','fast_only']);parser.add_argument('--prepare-only',action='store_true');a=parser.parse_args()
    out=CASE/a.group
    if not out.exists():prepare(a.group)
    if not a.prepare_only:runner.run(a.group,45.,11)
