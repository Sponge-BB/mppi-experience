import ast,hashlib,json,os,re,shutil,stat,subprocess
from pathlib import Path
root=Path('/home/l/remani_ws');x=json.loads(Path('/tmp/remani_arm_box_version.json').read_text());case=Path(x['case'])
def sha(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
 return h.hexdigest()
checks={};runs={}
for group in ['native','mppi','fast_only']:
 d=case/group;m=json.loads((d/'manifest.json').read_text())
 for key in ['source_sha256','runtime_sha256']:
  for rel,digest in m[key].items():assert sha(d/rel)==digest,(group,rel)
 for name,digest in m['native_binaries_sha256'].items():assert sha(d/'ros_pkgs/online_remani_experiment'/name)==digest,(group,name)
 status=json.loads((d/'run_status.json').read_text());assert status['result_written'] and status['graph_audited'] and not status['stop_reason'] and not status['required_process_failure']
 assert json.loads((d/'graph_audit.json').read_text())['valid']
 assert json.loads((d/'geometry_precheck.json').read_text())['passed']
 assert json.loads((d/'runtime_geometry_precheck.json').read_text())['passed']
 result=json.loads((d/'result.json').read_text());assert result['box_runtime'] and result['box_runtime']['front_gap_m']==1.0
 runs[group]={'status':status,'reason':result['reason'],'goal_reached':result['goal_reached'],'physical_collision':result['collision_physical'],'min_arm_box_gap_m':result['min_arm_box_gap_m']}
checks['prepared_sources_configs_models_and_native_binaries_match']=True
for rel in x['files']:
 p=root/rel
 if p.suffix=='.py':ast.parse(p.read_text(),filename=rel)
checks['python_syntax']=True
html=(case/'report/comparison.html').read_text();scripts=re.findall(r'<script(?:\s[^>]*)?>(.*?)</script>',html,re.S)
# Last inline block is browser JS; the earlier block contains JSON data.
js=Path('/tmp/remani_arm_box_report.js');js.write_text(scripts[-1]);subprocess.run(['node','--check',str(js)],check=True)
checks['inline_javascript_syntax']=True
frozen=json.loads((case/'fast_only_audit.json').read_text());assert frozen.get('fast_only_claim_supported') is True
checks['first_reference_complete_and_zero_slow_planning_after_freeze']=True
# Production source/devel deployed before this experiment must still match.
expected={}
for ver in ['20261114-194013-dense-edt','20261114-195627-live-witness']:
 expected.update(json.loads((root/'rollback'/ver/'files.json').read_text()))
for rel,m in expected.items():
 p=root/rel;assert stat.S_IMODE(p.lstat().st_mode)==m['mode'],rel
 if m['type']=='file':assert sha(p)==m['sha256'],rel
 else:assert os.readlink(p)==m['target'],rel
checks['production_source_and_devel_unchanged']=True
summary=json.loads((case/'report/summary.json').read_text());assert summary['missing_groups']==[]
artifacts={str(p.relative_to(case)):dict(bytes=p.stat().st_size,sha256=sha(p)) for p in (case/'report').iterdir() if p.is_file()}
(case/'artifact_manifest.json').write_text(json.dumps(artifacts,indent=2)+'\n')
shutil.copy2(case/'report/summary.json',case/'result_summary.json')
v={'offline_passed':True,'hardware_validation':False,'purpose':'行驶中机械臂前方一米突然出现长方体：原生/MPPI/冻结慢层三组仿真及实测回放','checks':checks,'runs':runs,'fast_only_claim_supported':True,'physical_base_locked':False,'production_modified':False,'source_runtime_file_count':len(x['files']),'initial_baseline_expected_absent':True,'known_limits':['One seed/run per group; boxes placed by same relative 1m rule, with slightly different actual world coordinates. No general success-rate or performance ranking.','Numerical native sphere geometry and shared bounded execution plant, not Gazebo mesh contact or hardware. Native pose/q adapter is a declared simulation execution approximation.','Native physical collision is a retained negative result; raw desired command bus was not captured separately.','Fast-only freeze is an isolated capability ablation retaining mapping/MPPI collision checks, not a deployment recommendation.','Restore preserves previously existing immutable active-arm simulation inputs/builds. Experimental recorded data and original immutable dependency objects are not deleted or duplicated in the code delta.']}
(case/'validation_summary.json').write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n');print(json.dumps(v,ensure_ascii=False,indent=2))
