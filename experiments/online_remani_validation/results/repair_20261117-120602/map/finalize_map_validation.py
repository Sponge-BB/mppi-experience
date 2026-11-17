#!/usr/bin/python3
from pathlib import Path
import json,hashlib,statistics,difflib,os,stat,datetime,subprocess
P=Path(__file__).resolve().parent
def read(p):return json.loads(p.read_text())
def sha(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
 return h.hexdigest()
variants=['axis_layout','negative_subset','float32_scratch'];reports={v:read(P/('paired_'+v+'_t3')/'summary.json') for v in variants}
meta=read(P/'fixtures/metadata.json')
results=[]
for v,r in reports.items():
 frames=[]
 for f in r['per_frame']:
  a,b=f['variants']['baseline'],f['variants'][v]
  frames.append({'name':f['frame'],'group':f['group'],'voxels':f['voxels'],'baseline_compute_median_ms':a['compute_ms']['median'],'candidate_compute_median_ms':b['compute_ms']['median'],'baseline_compute_p95_ms':a['compute_ms']['p95'],'candidate_compute_p95_ms':b['compute_ms']['p95'],'reduction_percent':f['reduction_percent'],'baseline_peak_rss_MiB':a['peak_rss_MiB'],'candidate_peak_rss_MiB':b['peak_rss_MiB']})
 results.append({'variant':v,'decision':'SELECTED_EXPERIMENTAL_STAGED_NOT_DEPLOYED' if v=='float32_scratch' else 'REJECT_NOT_DEPLOYED','exact':r['exact_every_pair'],'threads':3,'warmup_pairs':1,'measured_pairs':5,'voxels_compared_per_pair':sum(f['voxels'] for f in meta['frames']),'real_crop_median_of_frame_reduction_percent':r['real_crop_median_reduction_percent'],'synthetic_reductions':r['synthetic_reductions'],'frames':frames})
performance={'created_utc':datetime.datetime.utcnow().isoformat()+'Z','scope':meta['scope'],'original_current_2M_worker_occupancy_available':False,'missing_input_evidence':read(P/'preparation.json')['missing_real_full_input_evidence'],'selected':'float32_scratch','combined_candidates':False,'inference_limit':'No online end-to-end gain or current full-size real occupancy performance established. Original 350ms validity threshold, cap, halos, occupancy and collision semantics unchanged.','results':results,'excluded_performance_run':'paired_negative_subset_t3_interference','excluded_reason':'Overlapped root SDK diagnostic replay; entire run excluded from timing conclusions, preserved for audit.','first_growth_caveat':'The first 2.208M synthetic_empty input grows buffers from 403440 cells in each fresh process; its 30.56% median gain includes allocation/fill savings and is not a steady-state EDT claim.','tail_caveat':'Five measured samples per frame are a screening dataset; float thick-scene p95 was 51.57ms vs 47.68ms baseline despite a lower median. No universal tail-latency claim.'}
(P/'map_performance_report.json').write_text(json.dumps(performance,indent=2)+'\n')
(P/'float32_scratch.decision.json').write_text(json.dumps({'decision':'SELECTED_EXPERIMENTAL_STAGED_NOT_DEPLOYED','selected_by':'root requested full-library clean build after third-candidate A/B','scope':meta['scope'],'real_crop_median_reduction_percent':reports['float32_scratch']['real_crop_median_reduction_percent'],'synthetic_reductions':reports['float32_scratch']['synthetic_reductions'],'real_vehicle_validated':False,'full_current_real_input_available':False},indent=2)+'\n')
source=read(P/'full_plan_env_source_manifest.json')
for rel,m in source['staged_source'].items():
 f=P/'full_plan_env_source'/rel
 if 'sha256' in m:assert sha(f)==m['sha256'],rel
for manifest in (P/'build').glob('*.inputs.after.sha256'):
 for line in manifest.read_text().splitlines():
  h,name=line.split(maxsplit=1);assert sha(Path(name))==h,name
build=read(P/'full_build.json');tests=read(P/'full_tests_results.json')
assert build['exit_code']==0 and build['stop_reason'] is None
assert len(tests['records'])==3 and all(r['exit_code']==0 and not r['stop_reason'] and r['gtest_xml']['failures']=='0' and r['gtest_xml']['errors']=='0' for r in tests['records'])
lib=P/'full_plan_env_devel/lib/libplan_env.so'
normal_env=os.environ.copy();normal_env['LD_LIBRARY_PATH']='/home/l/remani_ws/devel/lib:/opt/ros/noetic/lib'
for tool,args,out in [('readelf',['-d',str(lib)],'staged_library.dynamic.txt'),('ldd',[str(lib)],'staged_library.production_rpath.ldd.txt')]:
 text=subprocess.check_output([tool]+args,env=normal_env,text=True);(P/out).write_text(text)
 if tool=='ldd':assert 'not found' not in text
commands=read(P/'full_plan_env_build/compile_commands.json')
compiled=sorted({str(Path(c['file']).relative_to(P/'full_plan_env_source')) for c in commands if Path(c['file']).is_relative_to(P/'full_plan_env_source')}) if hasattr(Path,'is_relative_to') else sorted({str(Path(c['file']).relative_to(P/'full_plan_env_source')) for c in commands if str(c['file']).startswith(str(P/'full_plan_env_source'))})
for name in ['dynamic_point_buffer.cpp','esdf_worker.cpp','grid_map.cpp','raycast.cpp']:assert 'src/'+name in compiled
production=Path('/home/l/remani_ws/src/mm_remani_planner/plan_env')
validation={'created_utc':datetime.datetime.utcnow().isoformat()+'Z','status':'FULL_STAGED_LIBRARY_OFFLINE_VALIDATED_NOT_DEPLOYED','selected_candidate':'float32_scratch','source_manifest':'full_plan_env_source_manifest.json','staged_library':str(lib),'staged_library_sha256':sha(lib),'staged_library_mode':oct(stat.S_IMODE(lib.stat().st_mode)),'production_worker_source_matches_preserved_baseline':sha(production/'src/esdf_worker.cpp')==sha(P/'baseline/esdf_worker.cpp'),'production_worker_header_matches_preserved_baseline':sha(production/'include/plan_env/esdf_worker.h')==sha(P/'baseline/include/plan_env/esdf_worker.h'),'candidate_source_sha256':{n:sha(P/'full_plan_env_source'/n) for n in ['src/esdf_worker.cpp','include/plan_env/esdf_worker.h','test/test_esdf_worker.cpp']},'full_library_compiled_sources':compiled,'full_build':build,'full_tests':tests,'performance':'map_performance_report.json','real_vehicle_validated':False,'current_full_real_input_available':False,'changes_excluded':['axis_layout','negative_subset'],'runtime_threshold_halo_cap_collision_semantics_changed':False,'deployment_owner':'root; this agent wrote only experiment directories','patches':['float32_scratch.patch','float32_scratch_tests.patch']}
(P/'validation_summary.json').write_text(json.dumps(validation,indent=2)+'\n')
test_orig=(production/'test/test_esdf_worker.cpp').read_text().splitlines(True);test_new=(P/'test_float32_scratch.cpp').read_text().splitlines(True)
(P/'float32_scratch_tests.patch').write_text(''.join(difflib.unified_diff(test_orig,test_new,fromfile='a/src/mm_remani_planner/plan_env/test/test_esdf_worker.cpp',tofile='b/src/mm_remani_planner/plan_env/test/test_esdf_worker.cpp')))
lines=['地图实验最终结果（仅 staged，运行 src/devel 未由本 agent 修改）', '', '选择：float32_scratch 单项，完整新 plan_env 已干净构建并离线验证；根代理负责正式源码/库同步应用与版本归档。未实车验证，未证明在线卡顿已解决。', '']
for item in results:
 lines.append(item['variant']+': '+item['decision']+'；真实历史 403440 格 crop 的逐帧中位改善率中位数 %.3f%%。'%item['real_crop_median_of_frame_reduction_percent'])
 for f in item['frames']:
  if f['group']=='real_crop':continue
  lines.append('  %s（%d 格）compute 中位 %.3f -> %.3f ms，改善 %.3f%%；p95 %.3f -> %.3f ms。'%(f['name'],f['voxels'],f['baseline_compute_median_ms'],f['candidate_compute_median_ms'],f['reduction_percent'],f['baseline_compute_p95_ms'],f['candidate_compute_p95_ms']))
lines+=['','精确性：每项均 1 组预热+5 组交替顺序实测，22 帧每组共 19,703,040 格，全部逐值相等/无非有限值；保留每组每帧 sha256、比较结果及两帧代表完整 f64。独立候选测试 axis 17/17、negative 19/19、float 18/18。','证据范围：本轮 map_events 是 82×82×60 ESDF 输出，无完整约200万格原始 occupancy；不能反演/补零冒充。采用此前真实 occupancy crop 与明确标注的合成220.8万格输入。负向曾与 SDK 回放交叠的整组时序已排除并保留。','float 精确边界：仅 ∑(size−1)^2≤2^24，整数平方距离存 float 后精确恢复 double；+inf 映射回 double MAX；envelope、sqrt、result 均 double。超界回双精度，模式不变复用容量。','内存：220.8万格两份 scratch 理论减少约16.85 MiB；实际该轮 worker 峰值 baseline61.82MiB、float43.46MiB。候选编译/执行均受600MiB AS；全库构建另获1.4GiB AS授权，实测见下。','限制：空图30.56%包含首次扩容，不能当稳定正向主耗时收益。真实crop仅1.52%改善；随机稠密合成3.10%。厚块5次样本p95反而47.68->51.57ms，不宣称所有尾延迟改善。','完整构建：四个 plan_env 源从新 stage 重编，无旧 .o；耗时 %.3fs，峰值组 RSS %.3fMiB，可用内存最低 %.3fMiB。'%(build['elapsed_s'],build['peak_process_group_rss_MiB'],build['minimum_MemAvailable_MiB'])]
for r in tests['records']:lines.append('  %s：%s 项，failures=%s，errors=%s，%.3fs。'%(r['test'],r['gtest_xml']['tests'],r['gtest_xml']['failures'],r['gtest_xml']['errors'],r['elapsed_s']))
lines+=['libplan_env.so SHA256: '+sha(lib),'库 RPATH 为正式 devel/ROS；三套测试经 LD_LIBRARY_PATH 明确加载 staged 库，ldd/readelf 报告已保存。','部署材料：full_plan_env_source/src/esdf_worker.cpp、include/plan_env/esdf_worker.h、test/test_esdf_worker.cpp；完整库 full_plan_env_devel/lib/libplan_env.so；validation_summary.json/source manifest。','运行库未由本 agent 部署。350ms/halo/cap/碰撞语义未改。']
(P/'map_performance_report.txt').write_text('\n'.join(lines)+'\n')
manifest=[]
for f in sorted(P.rglob('*')):
 if f.is_symlink():manifest.append({'path':str(f.relative_to(P)),'mode':oct(stat.S_IMODE(f.lstat().st_mode)),'symlink':os.readlink(str(f))})
 elif f.is_file() and f.name!='artifact_manifest.json' and 'full_plan_env_build' not in f.parts and 'isolated_test_ros_' not in str(f):manifest.append({'path':str(f.relative_to(P)),'mode':oct(stat.S_IMODE(f.stat().st_mode)),'bytes':f.stat().st_size,'sha256':sha(f)})
(P/'artifact_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
print(json.dumps({'status':validation['status'],'library_sha256':validation['staged_library_sha256'],'tests':{r['test']:r['gtest_xml']['tests'] for r in tests['records']},'disk_MiB':sum(f.stat().st_size for f in P.rglob('*') if f.is_file())/1024**2}),flush=True)
