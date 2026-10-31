#!/usr/bin/env python3
"""Immutable SIL build: consistent static horizon and reviewed alternatives."""
import argparse
import json
from pathlib import Path
import shlex
import shutil
import subprocess
from build_native_v9 import HERE,sha,replace
from build_native_v10 import compile_source


def build(base,out,planner_build=None,planner_object_build=None):
    base=base.resolve();out=out.resolve()
    manifest=json.loads((base/'build_manifest.json').read_text())
    assert manifest.get('local_execution_owner')
    assert sha(base/'build/libonline_mppi.so')==manifest['mppi_library_sha256']
    out.mkdir(parents=True,exist_ok=False)
    shutil.copytree(base/'source',out/'source');(out/'build').mkdir()
    for name in ('remani_planner_node','mm_controller_node','dynamic_obstacle_tracker_node'):
        shutil.copy2(base/'build'/name,out/'build'/name)
    kernel=(base/'source/online_kernel.hpp').read_text()
    old='int horizon=std::max(1,std::min(19,int(std::floor((c.max_prediction-std::max(0.,z.t-z.stamp)-.01)/c.rdt))));'
    kernel=replace(kernel,old,'int horizon=z.os.empty()?30:std::max(1,std::min(19,int(std::floor((c.max_prediction-std::max(0.,z.t-z.stamp)-.01)/c.rdt))));',2)
    # A complete, progressing static detour may end recovery even while direct
    # reference following remains blocked. Preserve the existing dynamic policy.
    kernel=replace(kernel,'    if(!failed&&!recovering){',
        '    if(!failed&&(!recovering||(z.os.empty()&&avoidance_debug[5]>.02))){')
    kernel=replace(kernel,'if(base.merit.feasible&&base.continuations>0&&terminalOpen(base.merit))return original;',
        'if(base.merit.feasible&&base.continuations>0&&terminalOpen(base.merit)){if(z.os.empty()){recovering=false;recovery_clear=0;}return original;}')
    # Infeasible candidates never enter the weighted update or final ranking.
    # Stop their evaluation at the first hard failure; do not spend the rest
    # of a 3s rollout computing costs for an already rejected candidate.
    # Keep the nominal rollout complete for conflict timing diagnostics.
    kernel=replace(kernel,
        'Merit evaluate(S s,const Z&z,const Scene&sc,const std::vector<double>&a,const std::vector<V>&us)const{',
        'Merit evaluate(S s,const Z&z,const Scene&sc,const std::vector<double>&a,const std::vector<V>&us,bool prune=true)const{')
    kernel=replace(kernel,'auto nm=evaluate(s,z,sc,a,nominal);','auto nm=evaluate(s,z,sc,a,nominal,false);')
    kernel=replace(kernel,'        m.cost+=.5*c.rdt*40*std::pow(std::max(0.,.18-d),2);',
        '        m.cost+=.5*c.rdt*40*std::pow(std::max(0.,.18-d),2);\n        if(prune&&!m.feasible)return m;')
    # Preserve an already viable full static detour instead of performing a
    # second large primitive search for the same maneuver every 50ms. The exact
    # common feasibility evaluator and publication-time validator still apply.
    kernel=replace(kernel,
        'auto saved=warm;V5MPPI::plan(s,z,sc);candidates.push_back(warm);warm=saved;',
        '''auto saved=warm;V5MPPI::plan(s,z,sc);
      if(z.os.empty()&&route.path.size()>1&&usable(s)){
        // During a static detour, progress follows the validated connector,
        // not a blocked original chord. Collision checks are unchanged.
        auto detour=evaluate(s,z,route,arcs(route),warm);
        if(detour.feasible&&detour.progress>.02&&(detour.end_gap>=.05||detour.end_closing<=.005)){
          capture(warm,detour.cost,detour.min_gap,true,true);
          avoidance_debug={nm.min_gap,1.,1.,0.,detour.min_gap,detour.progress,detour.deficit,0.};
          decision_debug={2.,nm.conflict<1e8?nm.conflict:-1.,budget,nm.min_gap,double(committed_side),0.,1.,p.distance,
            detour.progress/(warm.size()*c.rdt),detour.cost,double(clear_count),detour.lateral};
          return warm.front();
        }
      }
      candidates.push_back(warm);warm=saved;''')
    # All static fallback candidates must use the same valid connector as the
    # primary proposal. Previously only the primary and outer recovery did so;
    # the intervening primitive/weighted-selection stage still used the blocked
    # native reference and could repeatedly overwrite a detour with no progress.
    start,marker,tail=kernel.partition('    std::vector<std::vector<V>> candidates{nominal};')
    assert marker
    selection,end,recovery=tail.partition('struct MPPI : V7MPPI {')
    assert end
    selection=replace(selection,'      candidates.push_back(warm);warm=saved;', '''      if(z.os.empty()&&route.path.size()>1&&usable(s)){
        score_scene=&route;score_arc=arcs(route);score_projection=project(s,route,score_arc);
        score_targets.clear();for(const auto&r:route.path)score_targets.push_back(r.p);
        sign=direction(route,score_projection.index);
        for(double scale:{1.,.55,.3}){
          S roll=s;int hint=score_projection.index;std::vector<V> guided;
          for(int k=0;k<horizon;++k){V u=bound(scale*guide(roll,route,score_arc,score_targets,hint),c);
            guided.push_back(u);roll=step(roll,u,c.rdt,c);}
          candidates.push_back(std::move(guided));
        }
      }
      candidates.push_back(warm);warm=saved;''')
    selection=selection.replace('evaluate(s,z,sc,a,','evaluate(s,z,*score_scene,score_arc,')
    selection=selection.replace('int hint=p.index','int hint=score_projection.index')
    selection=selection.replace('guide(roll,sc,a,targets,hint)','guide(roll,*score_scene,score_arc,score_targets,hint)')
    selection=selection.replace('project(roll,sc,a,hint)','project(roll,*score_scene,score_arc,hint)')
    kernel=start+'''    const Scene*score_scene=&sc;auto score_arc=a;auto score_projection=p;auto score_targets=targets;
'''+marker+selection+end+recovery
    head,marker,recovery=kernel.partition('struct MPPI : V7MPPI {')
    assert marker
    recovery=replace(recovery,'auto a=arcs(sc);auto p=project(s,sc,a);std::vector<V> targets;',
        '''const Scene&policy=z.os.empty()&&route.path.size()>1&&usable(s)?route:sc;
    auto a=arcs(policy);auto p=project(s,policy,a);std::vector<V> targets;''')
    recovery=replace(recovery,'for(const auto&r:sc.path)targets.push_back(r.p);',
        'for(const auto&r:policy.path)targets.push_back(r.p);')
    recovery=replace(recovery,'evaluate(s,z,sc,a,warm)','evaluate(s,z,policy,a,warm)')
    recovery=replace(recovery,'evaluate(s,z,sc,a,us)','evaluate(s,z,policy,a,us)')
    recovery=replace(recovery,'guide(roll,sc,a,targets,hint)','guide(roll,policy,a,targets,hint)')
    kernel=head+marker+recovery
    (out/'source/online_kernel.hpp').write_text(kernel)
    api=(base/'source/online_api.cpp').read_text()
    api=replace(api,'return h;}catch(...){return nullptr;}',
        'h->m.trace_enabled=true;return h;}catch(...){return nullptr;}')
    api=replace(api,'m.route.path.clear();m.published_route.clear();}',
        'm.route.path.clear();m.published_route.clear();m.recovering=false;m.recovery_clear=0;m.committed_side=0;m.clear_count=0;}')
    api+='\n'+(HERE/'execution_candidates.inc').read_text()
    (out/'source/online_api.cpp').write_text(api)
    parent=Path(manifest['parent_build']);cm=parent/'build/CMakeFiles/online_mppi.dir'
    link=shlex.split((cm/'link.txt').read_text())
    oldobj=next(s for s in link if s.endswith('online_api.cpp.o'))
    # Keep baseline compiler flags; speedup comes from pruning rejected work.
    flags=(cm/'flags.make').read_text()
    includes=next(s.split('=',1)[1].strip() for s in flags.splitlines() if s.startswith('CXX_INCLUDES ='))
    obj=str(out/'build/online_api.cpp.o')
    subprocess.run(['/usr/bin/c++','-std=gnu++17','-O2','-DNDEBUG','-fPIC',*shlex.split(includes),
        '-c',str(out/'source/online_api.cpp'),'-o',obj],check=True)
    link=[obj if s==oldobj else s for s in link]
    link[link.index('-o')+1]=str(out/'build/libonline_mppi.so')
    subprocess.run(link,cwd=str(parent/'build'),check=True)
    # A missing approved fast rollout must not also prevent the slow planner
    # from refreshing its blocked guide. This grants NO execution authority.
    cpp=(base/'source/remani_replan_fsm.cpp').read_text()
    cpp=replace(cpp,'''  if(!have_target_||exec_state_!=EXEC_TRAJ||local_execution_path_.poses.size()<2||
     local_execution_path_.header.frame_id!="world"||
     (now-local_execution_path_.header.stamp).toSec()<-.05||
     (now-local_execution_path_.header.stamp).toSec()>.20)return;''',
        '  if(!have_target_||exec_state_!=EXEC_TRAJ)return;')
    (out/'source/remani_replan_fsm.cpp').write_text(cpp)
    # Native search occasionally supplies an empty base seed after a failed
    # search. Do not enter RRT with no edge or an invalid random-index range.
    sample=(parent/'source/mm_remani_planner/path_searching/src/sample_mani_RRT.cpp').read_text()
    sample=replace(sample,'    t_list_container.clear();', '''    t_list_container.clear();
    if(car_state_list.size()<2 || t_list.size()+1!=car_state_list.size() ||
       singul_container.size()!=t_list.size()){
      ROS_WARN("[SampleMani][InvalidSeed] Reject base seed: nodes=%zu times=%zu directions=%zu.",
               car_state_list.size(),t_list.size(),singul_container.size());
      return false;
    }
    if(check_num_<=0 || car_state_list_check.size()<t_list.size()*size_t(check_num_) ||
       start_state.size()!=manipulator_dof_ || end_state.size()!=manipulator_dof_ ||
       !start_state.allFinite() || !end_state.allFinite())return false;
    for(const auto&state:car_state_list)if(!state.allFinite())return false;
    for(const auto&state:car_state_list_check)if(!state.allFinite())return false;
    for(double duration:t_list)if(!std::isfinite(duration)||duration<=0.)return false;''')
    sample=replace(sample,'  bool SampleMani::search(const Eigen::VectorXd &start_state, const Eigen::VectorXd &end_state){',
        '''  bool SampleMani::search(const Eigen::VectorXd &start_state, const Eigen::VectorXd &end_state){
    if(max_index_<2){have_path_=false;return false;}''')
    sample=replace(sample,'  ManiPathNodePtr SampleMani::getSampleNode(){',
        '''  ManiPathNodePtr SampleMani::getSampleNode(){
    if(max_index_<3)return nullptr;''')
    (out/'source/sample_mani_RRT.cpp').write_text(sample)
    if planner_build is not None:
        for name in ('remani_planner_node.cpp','remani_replan_fsm.cpp','plan_manage/remani_replan_fsm.h','sample_mani_RRT.cpp'):
            assert sha(out/'source'/name)==sha(planner_build/'source'/name),'planner reuse requires identical source'
        shutil.copy2(planner_build/'build/remani_planner_node',out/'build/remani_planner_node')
    else:
        cm=parent/'build/CMakeFiles/remani_planner_node.dir';link=shlex.split((cm/'link.txt').read_text())
        for name in ('remani_planner_node.cpp','remani_replan_fsm.cpp'):
            oldobj=next(s for s in link if s.endswith('/'+name+'.o'))
            if planner_object_build is not None:
                for source in (name,'plan_manage/remani_replan_fsm.h'):
                    assert sha(out/'source'/source)==sha(planner_object_build/'source'/source)
                obj=str(out/'build'/(name+'.o'))
                shutil.copy2(planner_object_build/'build'/(name+'.o'),obj)
            else:
                _,obj=compile_source(parent,out,'remani_planner_node',out/'source'/name,oldobj,out/'source')
            link=[obj if s==oldobj else s for s in link]
        _,sample_obj=compile_source(parent,out,'exp_search',out/'source/sample_mani_RRT.cpp','unused')
        archive=out/'build/libexp_search.a'
        shutil.copy2(parent/'build/libexp_search.a',archive)
        subprocess.run(['ar','r',str(archive),sample_obj],check=True)
        assert 'libexp_search.a' in link
        link=[str(archive) if s=='libexp_search.a' else s for s in link]
        link[link.index('-o')+1]=str(out/'build/remani_planner_node')
        subprocess.run(link,cwd=str(parent/'build'),check=True)
    manifest.update(controller_revision='native_mppi_v11_execution_candidates',execution_candidates=True,
        execution_parent=str(base),mppi_library_sha256=sha(out/'build/libonline_mppi.so'),
        controller_changes=['static proposals and recovery retain the same 3s horizon; CV horizon unchanged',
            'complete feasible static detour can leave recovery without waiting for blocked nominal guide',
            'preserve a checked progressing static detour before duplicate primitive search',
            'measure static detour/recovery progress along its checked local connector, then rejoin native guide',
            'static primitive proposals and weighted selection use that same connector objective',
            'stop scoring at first hard infeasibility; feasible candidate scores and nominal diagnostics unchanged',
            'blocked native guide can refresh even when no local rollout has passed; no motion permission implied',
            'reject malformed native base seeds before manipulator RRT and protect random index bounds',
            'reference change invalidates recovery/side state while retaining control warm start',
            'ranked feasible candidate export; accepted alternative becomes the next warm start'],
        isolated_source_sha256={str(p.relative_to(out/'source')):sha(p) for p in (out/'source').rglob('*') if p.is_file()},
        builder_sha256=sha(Path(__file__)))
    manifest['mppi_compiler_optimization']='-O2 (baseline flags, no fast-math)'
    (out/'build_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(out,flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base',type=Path,default=HERE/'builds/native_v10_06')
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--planner-build',type=Path,help='reuse native binary only after identical source hashes')
    parser.add_argument('--planner-object-build',type=Path,help='reuse unchanged FSM/node objects, same O2 flags and header')
    args=parser.parse_args();build(args.base,args.out,args.planner_build,args.planner_object_build)
