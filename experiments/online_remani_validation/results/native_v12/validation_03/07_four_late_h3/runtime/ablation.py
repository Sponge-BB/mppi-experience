"""Generate only the no-wait experimental planner variant; production untouched."""
import hashlib
import json
import sys
from pathlib import Path


def generate(build):
    build=Path(build);source=build/'source/mm_remani_planner/plan_manage/src/planner_manager.cpp'
    text=source.read_text();needle='      if (delta_replan_time > 0) ros::Duration(delta_replan_time).sleep();'
    assert text.count(needle)==1
    text=text.replace(needle,'      // Experiment ablation: publish before activation; native controller already waits for its stamp.\n      // No change to predicted splice state or late-candidate rejection.\n      (void)delta_replan_time;')
    dest=build/'source/planner_manager_no_wait.cpp';dest.write_text(text)
    (build/'no_wait_manifest.json').write_text(json.dumps({'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'variant_sha256':hashlib.sha256(text.encode()).hexdigest(),'change':'remove pre-publication sleep only; keep activation timestamp, boundary state, validation and stop guards unchanged'},indent=2)+'\n')


if __name__=='__main__':generate(sys.argv[1])
