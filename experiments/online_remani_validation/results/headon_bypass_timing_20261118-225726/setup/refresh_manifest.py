"""Record the actual isolated inputs before a trial; never runs ROS."""
from pathlib import Path
import json,hashlib,xml.etree.ElementTree as ET
CASE=Path(__file__).resolve().parents[1]
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def refresh(group):
 d=CASE/group;assert not (d/'result.json').exists(),'do not overwrite executed input manifests'
 m=json.loads((d/'manifest.json').read_text())
 m.update(time_coverage_repair=group!='native',observed_corridor_bypass_repair=group!='native',route_specific_horizon_repair=group!='native',requested_prediction_horizon_s=2.,dynamic_prediction_coverage_s=2.,delivery_reserve_s=.2,
  source_sha256={str(p.relative_to(d)):digest(p) for p in (d/'runtime').glob('*.py')},
  runtime_sha256={str(p.relative_to(d)):digest(p) for p in [d/'parameters.yaml',d/'experiment.launch',d/'active_arm_config.json',d/'scenario.json',d/'model/mm.urdf',*(d/'bin').iterdir()]})
 for p in (d/'ros_pkgs/online_remani_experiment').iterdir():
  if p.is_symlink():assert str(p.resolve()).startswith(str(d)),p
 m['actual_planner_sha256']=digest(d/'bin/remani_planner_node')
 m['native_binaries_sha256']={n:digest(d/'bin'/n) for n in ['remani_planner_node','mm_controller_node','dynamic_obstacle_tracker_node']}
 (d/'manifest.json').write_text(json.dumps(m,ensure_ascii=False,indent=2)+'\n')
if __name__=='__main__':
 import sys
 for g in sys.argv[1:]:refresh(g)
