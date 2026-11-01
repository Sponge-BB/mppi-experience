#!/usr/bin/env python3
"""Fresh fixed-arm SIL build; archived V12 and production binaries untouched."""
import argparse
import json
from pathlib import Path
import shlex
import shutil
import subprocess
from build_native_v9 import HERE,sha,replace
from build_native_v10 import compile_source


def inject_body(text,signature,body):
    begin=text.index(signature);brace=text.index('{',begin)
    return text[:brace+1]+'\n'+body+'\n'+text[brace+1:]


def transform_model(text):
    text='#include "shared_collision_ros.hpp"\n'+text
    text=inject_body(text,'void MMConfig::setParam(',
        '    shared_collision::initialize(nh);')
    for name in ('checkCarObsCollision','checkCarPlanningObsCollision','checkCarDynamicObsCollision',
                 'checkCarPredictedDynamicObsCollision'):
        dynamic='Dynamic' in name
        predicted='Predicted' in name
        body='''    if(shared_collision::enabled()){
      auto metric=shared_collision::pose(car_state,QUERY_TIME,DYNAMIC_ONLY);
      min_dist=metric.physical+mobile_base_check_radius_;
      VALIDITY
      return !metric.valid||metric.gap < -1e-9;
    }'''.replace('QUERY_TIME','query_time.toSec()' if predicted else '0.')
        body=body.replace('DYNAMIC_ONLY','true' if dynamic else 'false')
        body=body.replace('VALIDITY','query_valid=metric.valid;' if predicted else '')
        text=inject_body(text,'bool MMConfig::'+name+'(',body)
    for name in ('checkManiObsCollision','checkManiPlanningObsCollision','checkManiDynamicObsCollision',
                 'checkManiPredictedDynamicObsCollision'):
        body='''    if(shared_collision::enabled()){
      min_dist=1e6;bool valid_arm=shared_collision::armMatches(mani_state);
      VALIDITY
      // The shared body cover already contains the LOCKED arm projection.
      // Do not charge a second, incompatible environmental arm geometry.
      return !valid_arm;
    }'''.replace('VALIDITY','query_valid=valid_arm;' if 'Predicted' in name else '')
        text=inject_body(text,'bool MMConfig::'+name+'(',body)
    text=inject_body(text,'double MMConfig::getCarObstacleMinDistance(','''    if(shared_collision::enabled()){
      auto metric=shared_collision::pose(car_state);
      return metric.valid?metric.physical+mobile_base_check_radius_:0.;
    }''')
    text=inject_body(text,'bool MMConfig::checkPredictedDynamicObstacleCollision(','''    if(shared_collision::enabled()){
      auto metric=shared_collision::pose(car_state,query_time.toSec(),true);
      query_valid=metric.valid&&shared_collision::armMatches(mani_state);
      coll_type=metric.gap < -1e-9?0:-1;
      return query_valid&&metric.gap < -1e-9;
    }''')
    return text


def transform_optimizer(text):
    text='#include "shared_collision_ros.hpp"\n'+text
    text=inject_body(text,'bool PolyTrajOptimizer::OptimizeTrajectory_lbfgs(','''    shared_collision::ObjectiveFreeze collision_objective;
    if(shared_collision::enabled()&&!collision_objective.valid())return false;''')
    # Objective keeps one immutable measurement for the complete LBFGS solve.
    # Native planning/map state stays on its existing queue. The dedicated
    # collision receiver may atomically replace the latest measurement; the
    # thread-local objective guard keeps this solve's chosen frame unchanged.
    marker='    std::vector<Eigen::Vector3d> car_pts;\n    std::vector<Eigen::Matrix2d> car_dPtsdv_list;'
    text=replace(text,marker,'''    const bool unified=shared_collision::enabled();
    if(unified){
      Eigen::Matrix2d orientation=mm_config_->calR(vel.head(2),singul_container_[trajid]);
      double yaw=std::atan2(orientation(1,0),orientation(0,0));
      auto metric=shared_collision::pose(Eigen::Vector3d(pos(0),pos(1),yaw));
      if(!metric.valid)throw std::runtime_error("unavailable shared collision input during optimization");
      costp=metric.cost;ret=costp>0;
      gradp.head(2)=Eigen::Vector2d(metric.gx,metric.gy);
      gradv.head(2)=metric.gyaw*mm_config_->caldYawdV(vel.head(2));
    }
'''+marker)
    text=replace(text,'for(unsigned int i = 0; i < car_pts.size(); ++i){',
        'for(unsigned int i = 0; !unified && i < car_pts.size(); ++i){')
    text=replace(text,'''        grid_map_->evaluatePlanningEDTWithGrad(
            pt_on_link, dist, dist_grad);''','''        if(!unified)grid_map_->evaluatePlanningEDTWithGrad(pt_on_link,dist,dist_grad);
        else {dist=1e6;dist_grad.setZero();}''')
    return text


def transform_kernel(text,rejoin=True):
    text='#include "shared_collision.hpp"\n'+text
    text=replace(text,'void feedback(V actual){double fraction=',
        'void feedback(V actual){if(warm.empty())return;double fraction=')
    anchor='double clearance(const S&s,const std::vector<O>&os,const C&c,double ymax,double margin=0){'
    helper='''shared_collision::Evaluation commonMetric(const S&s,const std::vector<O>&os){
  auto model=shared_collision::snapshot();
  if(!model)throw std::runtime_error("unbound shared collision input");
  if(os.size()>16)throw std::runtime_error("too many collision targets");
  // Clearance and soft merit ask the SAME immutable pose/forecast twice.
  // Retain the shared_ptr as well as exact coordinates: pointer reuse or a
  // new map binding can never turn this small cache into stale collision data.
  struct Cache{std::shared_ptr<const shared_collision::Snapshot>model;
    S pose{};std::vector<O>obstacles;shared_collision::Evaluation value;};
  static thread_local Cache cache;
  bool same=cache.model==model&&cache.pose.x==s.x&&cache.pose.y==s.y&&cache.pose.yaw==s.yaw&&cache.obstacles.size()==os.size();
  for(size_t i=0;same&&i<os.size();++i)same=cache.obstacles[i].p.x==os[i].p.x&&
    cache.obstacles[i].p.y==os[i].p.y&&cache.obstacles[i].r==os[i].r&&
    cache.obstacles[i].vel.x==os[i].vel.x&&cache.obstacles[i].vel.y==os[i].vel.y&&cache.obstacles[i].moving==os[i].moving;
  if(same)return cache.value;
  std::array<shared_collision::Obstacle,16>values;
  for(size_t i=0;i<os.size();++i){const auto&o=os[i];values[i]={o.p.x,o.p.y,o.vel.x,o.vel.y,o.r,double(o.moving)};}
  cache.model=model;cache.pose=s;cache.obstacles=os;
  cache.value=shared_collision::evaluate(*model,s.x,s.y,s.yaw,0.,true,true,values.data(),os.size());
  return cache.value;
}
double commonCost(const S&s,const std::vector<O>&os){return commonMetric(s,os).cost;}
'''
    text=replace(text,anchor,helper+anchor+'''
  if(shared_collision::enabled())return commonMetric(s,os).physical-margin;''')
    text=replace(text,'m.cost+=.5*c.rdt*40*std::pow(std::max(0.,.18-d),2);',
                 'm.cost+=.5*c.rdt*commonCost(s,predict(z,t,c));')
    text=replace(text,'''    m.cost+=200*std::pow(std::max(0.,.03+closing*response-last_gap),2)
      +30*std::pow(std::max(0.,.18-last_gap),2);''','''    // Collision merit is shared; braking remains an independent hard check.
    // No extra terminal clearance objective attracting motion off the guide.''')
    text=replace(text,'double risk_cost=2*std::pow(clip(-risk,0.,.5),2)+15*near*near;',
        'double risk_cost=commonCost(s,nominal[k]);')
    text=replace(text,'cost+=c.rdt*(2*p.distance*p.distance+10*reverse*reverse+20*std::pow(std::max(0.,.04-d),2));',
        'cost+=c.rdt*(2*p.distance*p.distance+10*reverse*reverse+commonCost(s,predict(z,(k+1)*c.rdt,c)));')
    if rejoin:
        text=replace(text,'m.cost+=c.rdt*(5*p.distance*p.distance+.6*std::pow(wrap(s.yaw-p.yaw),2)+1.5*deficit);',
            'm.cost+=c.rdt*(80*p.distance*p.distance+.6*std::pow(wrap(s.yaw-p.yaw),2)+1.5*deficit);')
        text=replace(text,'m.cost+=12*p.distance*p.distance+.5*std::pow(wrap(s.yaw-p.yaw),2)-12*m.progress;',
            'm.cost+=35*p.distance*p.distance+.5*std::pow(wrap(s.yaw-p.yaw),2)-12*m.progress;')
    return text


def transform_v12(text,rejoin=True):
    text=replace(text,'void referenceChanged(){d_active=false;d_clear=0;committed_side=0;}',
        'void referenceChanged(){warm.clear();d_active=false;d_clear=0;committed_side=0;}')
    text=replace(text,'m.cost+=c.rdt/2*70*std::pow(std::max(0.,.25-gap),2);',
        'm.cost+=c.rdt/2*commonCost(s,future[2*k+j+1]);')
    text=replace(text,'''    m.cost+=250*std::pow(std::max(0.,.06+closing*response-last),2)+
      60*std::pow(std::max(0.,.22-last),2);''',
        '''    // SPACE cost is common. TIME-tail closing risk is a separate
    // dynamic-only term, not a hidden static clearance band. A safe static
    // replacement reference must not be repelled by a far moving target.
    if(!future.back().empty()){
      double final_dynamic=dynamic_tail_gap,closing_dynamic=dynamic_tail_closing;
      m.cost+=250*std::pow(std::max(0.,.06+closing_dynamic*response-final_dynamic),2);
    }
''')
    text=replace(text,'double start=p.arc,last=1e9,closing=0;V prev{s.v,s.w};',
        'double start=p.arc,last=1e9,closing=0,dynamic_tail_gap=1e9,dynamic_tail_closing=0;V prev{s.v,s.w};')
    text=replace(text,'closing=last<1e8?std::max(0.,(last-gap)/(c.rdt/2)):0.;last=gap;',
        '''closing=last<1e8?std::max(0.,(last-gap)/(c.rdt/2)):0.;last=gap;
        double next_dynamic=1e9;
        for(const auto&b:c.body)for(const auto&o:future[2*k+j+1])if(o.moving)
          next_dynamic=std::min(next_dynamic,norm(bodypoint(s,b.p)-o.p)-b.r-o.r-c.margin);
        dynamic_tail_closing=dynamic_tail_gap<1e8?std::max(0.,(dynamic_tail_gap-next_dynamic)/(c.rdt/2)):0.;
        dynamic_tail_gap=next_dynamic;''')
    if rejoin:
        text=replace(text,'bool goalCapture(const S&s,const Scene&sc)const{\n    if(sc.path.size()<2)return false;',
            '''bool goalCapture(const S&s,const Scene&sc)const{
    if(sc.path.size()<2)return false;
    auto lengths=arcs(sc);auto projection=project(s,sc,lengths);
    if(lengths.back()-projection.arc>.35)return false;''')
        text=replace(text,'double adherence=endpoint?0.:(side_pass?.35:5.);',
            'double adherence=endpoint?0.:(side_pass?.35:80.);')
        text=replace(text,'(side_pass?1.5:12.)*p.distance*p.distance;',
            '(side_pass?1.5:35.)*p.distance*p.distance;')
    return text


def compile_archive(parent,out,target,filename,archive_name):
    source=out/'source'/filename
    _,obj=compile_source(parent,out,target,source,'unused',out/'source')
    archive=out/'build'/archive_name;shutil.copy2(parent/'build'/archive_name,archive)
    subprocess.run(['ar','r',str(archive),obj],check=True)
    return archive


def build(base,out,rejoin=True):
    base=base.resolve();out=out.resolve();m=json.loads((base/'build_manifest.json').read_text())
    assert m['v12_controller'] and sha(base/'build/libonline_mppi.so')==m['mppi_library_sha256']
    parent=Path(m['parent_build']);out.mkdir(parents=True,exist_ok=False)
    shutil.copytree(base/'source',out/'source');(out/'build').mkdir()
    for name in ('shared_collision.hpp','shared_collision_ros.hpp'):shutil.copy2(HERE/name,out/'source'/name)
    for name in ('mm_controller_node','dynamic_obstacle_tracker_node'):shutil.copy2(base/'build'/name,out/'build'/name)
    fsm='#include "shared_collision_ros.hpp"\n'+(base/'source/remani_replan_fsm.cpp').read_text()
    for signature in ('bool REMANIReplanFSM::planFromGlobalTraj(', 'bool REMANIReplanFSM::planFromLocalTraj('):
        fsm=inject_body(fsm,signature,'''    shared_collision::ObjectiveFreeze collision_plan_boundary;
    if(shared_collision::enabled()&&!collision_plan_boundary.valid())return false;''')
    marker='      poly_traj_pub_.publish(msg);'
    fsm=replace(fsm,marker,'''      if(shared_collision::enabled()){
        auto collision=shared_collision::objectiveSnapshot()?shared_collision::objectiveSnapshot():shared_collision::snapshot();
        if(collision)ROS_INFO("[SharedCollision] native_reference sequence=%.0f input_age=%.3f",collision->sequence,ros::Time::now().toSec()-collision->stamp);
      }
'''+marker)
    (out/'source/remani_replan_fsm.cpp').write_text(fsm)
    original=parent/'source/mm_remani_planner'
    (out/'source/mm_config.cpp').write_text(transform_model((original/'mm_config/src/mm_config.cpp').read_text()))
    (out/'source/poly_traj_optimizer.cpp').write_text(transform_optimizer((original/'traj_opt/src/poly_traj_optimizer.cpp').read_text()))
    model=compile_archive(parent,out,'exp_model','mm_config.cpp','libexp_model.a')
    optimizer=compile_archive(parent,out,'exp_opt','poly_traj_optimizer.cpp','libexp_opt.a')
    # Preserve the reviewed V11 malformed-seed protections, not the old archive.
    _,search_object=compile_source(parent,out,'exp_search',out/'source/sample_mani_RRT.cpp','unused',out/'source')
    search=out/'build/libexp_search.a';shutil.copy2(parent/'build/libexp_search.a',search)
    subprocess.run(['ar','r',str(search),search_object],check=True)
    link=shlex.split((parent/'build/CMakeFiles/remani_planner_node.dir/link.txt').read_text())
    for name in ('remani_planner_node.cpp','remani_replan_fsm.cpp'):
        old=next(s for s in link if s.endswith('/'+name+'.o'))
        _,obj=compile_source(parent,out,'remani_planner_node',out/'source'/name,old,out/'source')
        link=[obj if s==old else s for s in link]
    link=[str(model) if s=='libexp_model.a' else str(optimizer) if s=='libexp_opt.a'
          else str(search) if s=='libexp_search.a' else s for s in link]
    link[link.index('-o')+1]=str(out/'build/remani_planner_node')
    subprocess.run(link,cwd=str(parent/'build'),check=True)
    kernel=transform_kernel((base/'source/online_kernel.hpp').read_text(),rejoin)
    (out/'source/online_kernel.hpp').write_text(kernel)
    (out/'source/mppi_v12.hpp').write_text(transform_v12((base/'source/mppi_v12.hpp').read_text(),rejoin))
    api=(base/'source/online_api.cpp').read_text()
    api=replace(api,'auto&m=static_cast<Online*>(p)->m;m.route.path.clear();',
        'auto&m=static_cast<Online*>(p)->m;m.side=0;m.route.path.clear();')
    api+='''
extern "C" int online_set_collision_snapshot(void*p,const double*data,int count){
  if(!p||!data||count<16)return 1;
  auto&c=static_cast<Online*>(p)->c;
  if(std::abs(data[8]-c.margin)>1e-9||data[11]!=c.body.size()||count<16+3*int(c.body.size()))return 3;
  for(size_t i=0;i<c.body.size();++i)if(std::abs(data[16+3*i]-c.body[i].p.x)>1e-9||
      std::abs(data[17+3*i]-c.body[i].p.y)>1e-9||std::abs(data[18+3*i]-c.body[i].r)>1e-9)return 3;
  shared_collision::enabled()=true;
  return shared_collision::bind(data,count)?0:2;
}
extern "C" int online_common_pose(double x,double y,double yaw,double forecast,double*out){
  auto model=shared_collision::snapshot();if(!model)return 1;
  auto value=shared_collision::evaluate(*model,x,y,yaw,forecast);
  double v[]={value.physical,value.gap,value.cost,value.gx,value.gy,value.gyaw,model->sequence};
  std::copy(v,v+7,out);return value.valid?0:2;
}
'''
    (out/'source/online_api.cpp').write_text(api)
    link=shlex.split((parent/'build/CMakeFiles/online_mppi.dir/link.txt').read_text())
    old=next(s for s in link if s.endswith('/online_api.cpp.o'))
    _,obj=compile_source(parent,out,'online_mppi',out/'source/online_api.cpp',old,out/'source')
    link=[obj if s==old else s for s in link];link[link.index('-o')+1]=str(out/'build/libonline_mppi.so')
    subprocess.run(link,cwd=str(parent/'build'),check=True)
    m.update(controller_revision='v12_shared_collision_rejoin' if rejoin else 'v12_shared_collision_only',
        shared_collision=True,shared_rejoin=rejoin,execution_parent=str(base),
        shared_collision_source_sha256=sha(out/'source/shared_collision.hpp'),
        experiment_parameter_overrides={**m['experiment_parameter_overrides'],
          '/remani_planner_node/fsm/unified_projection_collision':True},
        mppi_library_sha256=sha(out/'build/libonline_mppi.so'),builder_sha256=sha(Path(__file__)),
        isolated_source_sha256={str(p.relative_to(out/'source')):sha(p) for p in (out/'source').rglob('*') if p.is_file()},
        native_binaries_sha256={name:sha(out/'build'/name) for name in ('remani_planner_node','mm_controller_node','dynamic_obstacle_tracker_node')})
    (out/'build_manifest.json').write_text(json.dumps(m,indent=2)+'\n');print('BUILT',out,flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--base',type=Path,default=HERE/'builds/native_v12_07')
    p.add_argument('--out',type=Path,required=True);p.add_argument('--collision-only',action='store_true')
    a=p.parse_args();build(a.base,a.out,not a.collision_only)
