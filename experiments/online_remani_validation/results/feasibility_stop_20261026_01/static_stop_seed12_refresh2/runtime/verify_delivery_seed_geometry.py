#!/usr/bin/env python3
"""Extract exact native geometry/time functions and run a standalone Eigen test.

Never changes production, baseline or isolated build files. Newly generated
headers/binary/logs live only in a fresh result directory. No ROS/link to ROS.
"""
import argparse
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import re
import subprocess

from report_domain_recovery import assert_plot_quiet

HERE=Path(__file__).resolve().parent
WORKSPACE=HERE.parent.parent
FUNCTIONS=('getTrajsWithTime','evaluateCurvatureWeightedDistance','evaluateDuration','evaluateLength','getFlatState')


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def textsha(value):return hashlib.sha256(value.encode()).hexdigest()


def extract_function(text,name):
    matches=list(re.finditer(r'^(?:void|double) KinoAstar::'+re.escape(name)+r'\(',text,re.M))
    if len(matches)!=1:raise ValueError('Function missing/ambiguous: '+name)
    start=matches[0].start();opening=text.index('{',start);depth=0;state='code';i=opening
    while i<len(text):
        c=text[i];n=text[i+1:i+2]
        if state=='line':
            if c=='\n':state='code'
        elif state=='block':
            if c=='*' and n=='/':state='code';i+=1
        elif state in ('"',"'"):
            if c=='\\':i+=1
            elif c==state:state='code'
        elif c=='/' and n=='/':state='line';i+=1
        elif c=='/' and n=='*':state='block';i+=1
        elif c in ('"',"'"):state=c
        elif c=='{':depth+=1
        elif c=='}':
            depth-=1
            if depth==0:
                end=i+1;return dict(name=name,text=text[start:end],start_byte=start,end_byte=end,
                    first_line=text.count('\n',0,start)+1,last_line=text.count('\n',0,end)+1,
                    sha256=textsha(text[start:end]))
        i+=1
    raise ValueError('Unterminated function: '+name)


CLASS='''
class KinoAstar {
public:
 double max_vel_=0.,max_acc_=0.,min_turning_radius_=0.,curvatureDisCoe_=0.;
 double non_siguav_=0.,mobile_base_wheel_base_=0.,sample_time_=0.,totalTrajTime=0.;
 Eigen::Vector4d start_state_,end_state_;Eigen::Vector2d start_ctrl_;
 std::vector<Eigen::Vector3d> sample_trajs_;
 std::vector<double> shot_lengthList,shot_timeList;
 std::vector<int> shotindex,shot_SList;
 std::vector<CarFlatTrajData> flat_trajs_;
 void getTrajsWithTime();
 double evaluateCurvatureWeightedDistance(const Eigen::Vector2d&,const Eigen::Vector2d&,const double&,const double&);
 double evaluateDuration(const double&,const double&,const double&);
 double evaluateLength(const double&,const double&,const double&,const double&,const double&);
 void getFlatState(const Eigen::Vector4d&,const Eigen::Vector2d&,const int&,Eigen::MatrixXd&);
};
'''


def parameter_values(path):
    values={}
    for line in path.read_text().splitlines():
        if line.startswith('/remani_planner_node/') and ':' in line:
            key,value=line.split(':',1)
            try:values[key.split('/remani_planner_node/',1)[1]]=float(value)
            except ValueError:pass
    keys=('mm/mobile_base_max_wheel_omega','mm/mobile_base_max_wheel_alpha','mm/mobile_base_wheel_radius',
          'mm/mobile_base_non_singul_vel','mm/mobile_base_wheel_base','search/min_turning_radius',
          'search/curvatureDisCoe','search/sample_time')
    if any(k not in values for k in keys):raise ValueError('Missing native scalar parameter')
    p={k:values[k] for k in keys}
    return dict(max_vel_=p[keys[0]]*p[keys[2]],max_acc_=p[keys[1]]*p[keys[2]],
        non_siguav_=p[keys[3]],mobile_base_wheel_base_=p[keys[4]],min_turning_radius_=p[keys[5]],
        curvatureDisCoe_=p[keys[6]],sample_time_=p[keys[7]])


def verify(build,batch,out):
    build,batch,out=build.resolve(),batch.resolve(),out.resolve()
    complete=json.loads((batch/'complete.json').read_text());jobs=complete.get('completed')
    if complete.get('complete') is not True or not jobs:raise ValueError('Completed measurement batch required')
    assert_plot_quiet(dict(trials=[dict(run_status=json.loads((batch/j['trial']/'run_status.json').read_text())) for j in jobs]))
    manifest=json.loads((build/'build_manifest.json').read_text());source=build/'source/kino_astar.cpp'
    if sha(source)!=manifest['isolated_source_sha256']['kino_astar.cpp']:raise ValueError('Unattested search source')
    header=WORKSPACE/'src/mm_remani_planner/path_searching/include/path_searching/kino_astar.h'
    container=WORKSPACE/'src/mm_remani_planner/traj_utils/include/traj_utils/plan_container.hpp'
    for path in (header,container):
        if sha(path)!=manifest['production_sources_sha256'][str(path.relative_to(WORKSPACE))]:
            raise ValueError('Native header differs from production attestation')
    macro=re.findall(r'^#define inf[^\n]*',header.read_text(),re.M)
    if len(macro)!=1:raise ValueError('Missing/ambiguous native inf macro')
    functions=[extract_function(source.read_text(),name) for name in FUNCTIONS]
    struct=re.search(r'  struct CarFlatTrajData\{.*?\n  \};',container.read_text(),re.S)
    if not struct:raise ValueError('Missing exact native data struct')
    parameter_path=batch/'static_capture_seed11/parameters.yaml';parameters=parameter_values(parameter_path)
    if parameters!=parameter_values(batch/'static_capture_seed12/parameters.yaml'):
        raise ValueError('Online scalar parameters differ')
    out.mkdir(parents=True,exist_ok=False)
    generated='using namespace Eigen;\nusing namespace std;\n'+macro[0]+'\n'+struct.group()+'\n'+CLASS
    generated+='\n\n'.join(f['text'] for f in functions)+'\n'
    (out/'native_functions.hpp').write_text(generated)
    fixture='void runCase(const std::string&,const std::vector<Eigen::Vector3d>&,double);\n'
    fixture+='std::vector<Eigen::Vector3d> straight(const Eigen::Vector3d&,const std::vector<double>&);\n'
    fixture+='void configure(KinoAstar& s){\n'+''.join('s.%s=%.17g;\n'%(k,v) for k,v in parameters.items())+'}\n'
    fixture+='void runCapturedBoundaryConstructedCases(){\n';constructed=[]
    for seed in (11,12):
        trial=batch/('static_capture_seed%d'%seed);timeline=trial/'dual_timeline.jsonl'
        rows=[json.loads(line) for line in timeline.read_text().splitlines() if line]
        match=[r for r in rows if r.get('event')=='delivery_search_exit' and r.get('solver')=='kino_astar'
               and r.get('request_id')==3]
        if len(match)!=1:raise ValueError('Expected one captured request3 A* boundary')
        state=match[0]['start_state'];q=','.join('%.17g'%x for x in state[:3])
        name='constructed_reverse_forward_at_seed%d_request3_boundary'%seed
        fixture+='runCase("%s",straight({%s},{0.,-.01,-.02,-.01,0.,.01}),%.17g);\n'%(name,q,state[3])
        constructed.append(dict(name=name,boundary_start_state=state,request_id=3,pid=match[0]['pid'],
            source=str(timeline),source_sha256=sha(timeline),historical_path_replayed=False))
    fixture+='}\n';(out/'native_fixture.hpp').write_text(fixture)
    command=['/usr/bin/c++','-std=c++17','-O2','-Wall','-Wextra','-pedantic','-I/usr/include/eigen3',
        '-I'+str(out),str(HERE/'test_delivery_seed_geometry.cpp'),'-o',str(out/'test_delivery_seed_geometry')]
    with (out/'compile.log').open('w') as log:compiled=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT)
    if compiled.returncode:raise RuntimeError('Harness compile failed; retained '+str(out))
    with (out/'harness_stdout.jsonl').open('w') as stdout,(out/'harness_stderr.log').open('w') as stderr:
        ran=subprocess.run([str(out/'test_delivery_seed_geometry')],stdout=stdout,stderr=stderr,timeout=30)
    if ran.returncode:raise RuntimeError('Harness assertions failed; retained '+str(out))
    cases=[json.loads(line) for line in (out/'harness_stdout.jsonl').read_text().splitlines()]
    if len(cases)!=6:raise ValueError('Incomplete constructed fixture set')
    result=dict(schema='delivery_seed_geometry_v1',created=datetime.now(timezone.utc).isoformat(),
        build=str(build),build_manifest_sha256=sha(build/'build_manifest.json'),native_source=str(source),native_source_sha256=sha(source),
        native_header=str(header),native_header_sha256=sha(header),native_struct_header=str(container),native_struct_header_sha256=sha(container),
        native_macro=macro[0],functions=functions,parameters=parameters,parameters_source=str(parameter_path),parameters_source_sha256=sha(parameter_path),
        generated_header_sha256=sha(out/'native_functions.hpp'),generated_fixture_sha256=sha(out/'native_fixture.hpp'),
        harness_sha256=sha(HERE/'test_delivery_seed_geometry.cpp'),verifier_sha256=sha(Path(__file__)),
        compiler_command=command,compile_returncode=compiled.returncode,harness_returncode=ran.returncode,
        binary_sha256=sha(out/'test_delivery_seed_geometry'),constructed_boundary_fixtures=constructed,cases=cases,
        equivalence_scope='Exact five native function texts and native macro/struct, same scalar parameters and Eigen. Minimal class only changes visibility and excludes unused ROS/search/map members. ROS_ERROR is stderr stub; valid test speeds never enter error branch.',
        historical_full_path_replayed=False,baseline_or_production_modified=False,ros_started=False,
        limitations=['构造几何验证，不是历史两例完整路径重放；未执行原生A*、地图碰撞、机械臂采样或闭环导航。',
            '空flat块的flat_outputs_finite为真是空集结果，不能解释为产生了有效轨迹；必须同时检查块数量。',
            '只验证原函数问题，不在本测试修改宏、距离计算、时间分配或安全约束。'])
    (out/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    lines=['底盘种子几何与时间参数化：离线原函数验证','',
        '精确提取5个原函数和原inf宏/数据struct；仅构造输入，不是历史路径完整重放。','']
    for c in cases:lines.append('%s：欧氏长度=%s；原加权长度=%s；提前返回=%s；flat块=%s；输出全有限=%s'%(c['name'],
        c['euclidean_length'],c['weighted_length'],c['parameterization_early_return'],c['flat_block_count'],c['flat_outputs_finite']))
    lines+=['','原始宏：'+macro[0]+'；编译运行实际值：'+str(cases[0]['native_inf_macro_value']),
        '不可只修宏后直接上线：仍需处理反向距离零值、零长度段及参数化后的有效性。','',result['equivalence_scope']]+result['limitations']
    (out/'实验结论.txt').write_text('\n'.join(lines)+'\n')
    print(json.dumps(dict(out=str(out),cases=cases),ensure_ascii=False,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--build',type=Path,default=HERE/'builds/delivery_stability_20261024_02')
    p.add_argument('--batch',type=Path,default=HERE/'results/delivery_stability_20261024_01')
    p.add_argument('--out',type=Path,required=True);a=p.parse_args();verify(a.build,a.batch,a.out)
