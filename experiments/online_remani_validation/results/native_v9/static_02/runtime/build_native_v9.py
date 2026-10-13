#!/usr/bin/env python3
"""Immutable isolated build: atomic detector labels + unified V8 static search."""
import argparse
import hashlib
import json
from pathlib import Path
import shlex
import shutil
import subprocess

HERE=Path(__file__).resolve().parent
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def replace(text,old,new,count=1):
    if text.count(old)!=count:raise RuntimeError('snapshot marker changed: '+old[:100])
    return text.replace(old,new)


def build(base,out):
    base=base.resolve();out=out.resolve()
    m=json.loads((base/'build_manifest.json').read_text());parent=Path(m['parent_build'])
    assert sha(base/'build/libonline_mppi.so')==m['mppi_library_sha256']
    out.mkdir(parents=True,exist_ok=False);(out/'source').mkdir();(out/'build').mkdir()
    for n in ('remani_planner_node','remani_planner_node_no_wait','mm_controller_node'):
        shutil.copy2(base/'build'/n,out/'build'/n)
    kernel=(base/'source/online_kernel.hpp').read_text()
    # V7's common rollout evaluator already checks the map without dynamic tracks.
    kernel=replace(kernel,'    if(z.os.empty()){committed_side=clear_count=0;return V5MPPI::plan(s,z,sc);}\n','')
    kernel=replace(kernel,'if(!z.fresh||sc.path.size()<2||z.os.empty()){recovering=false;recovery_clear=0;return original;}',
                   'if(!z.fresh||sc.path.size()<2){recovering=false;recovery_clear=0;return original;}')
    # A newly revealed static obstacle can place the measured initial state
    # inside the preferred margin. Do NOT treat all such states as collisions.
    # Recovery is permitted only with positive conservative body clearance,
    # monotonically increasing clearance until margin restoration, and a
    # terminal state back inside the original feasible set. No CV relaxation.
    kernel=replace(kernel,'    Merit m;auto p=project(s,sc,a);double initial=p.arc;V previous{s.v,s.w};double last_gap=1e9,closing=0;',
        '''    Merit m;auto p=project(s,sc,a);double initial=p.arc;V previous{s.v,s.w};double last_gap=1e9,closing=0;
    double previous_gap=clearance(s,predict(z,0,c),c,sc.ymax,c.margin);
    bool restore=z.os.empty() && previous_gap<0 && previous_gap+c.margin>=.02;
    const bool started_restoration=restore;''')
    kernel=replace(kernel,'        m.min_gap=std::min(m.min_gap,d);if(d<0)m.feasible=false;',
        '''        m.min_gap=std::min(m.min_gap,d);
        if(restore){
          if(d<previous_gap-1e-7 || d+c.margin<.02)m.feasible=false;
          if(d>=0)restore=false;
        }else if(d<0)m.feasible=false;
        previous_gap=d;''')
    kernel=replace(kernel,'    m.end_gap=last_gap;m.end_closing=closing;',
        '''    if(started_restoration && last_gap<0)m.feasible=false;
    m.end_gap=last_gap;m.end_closing=closing;''')
    kernel=replace(kernel,'  std::array<double,10> recovery_debug{};',
        '  std::array<double,10> recovery_debug{};\n  std::array<double,4> margin_debug{};')
    kernel=replace(kernel,'    recovery_debug={};auto previous=warm;V original=V7MPPI::plan(s,z,sc);',
        '''    recovery_debug={};auto previous=warm;V original=V7MPPI::plan(s,z,sc);
    double static_gap=clearance(s,{},c,sc.ymax,c.margin);
    bool restoring=z.os.empty() && static_gap<0 && static_gap+c.margin>=.02;
    margin_debug={static_gap,double(restoring),double(restoring && avoidance_debug[7]==0),0.};''')
    kernel=replace(kernel,'    auto&selected=options[best];warm=selected.us;',
        '    auto&selected=options[best];warm=selected.us;\n    margin_debug[2]=restoring; margin_debug[3]=selected.merit.end_gap;')
    kernel=replace(kernel,'recovery_debug={};recovering=false;recovery_clear=0;}',
        'recovery_debug={};margin_debug={};recovering=false;recovery_clear=0;}')
    (out/'source/online_kernel.hpp').write_text(kernel)
    api=(base/'source/online_api.cpp').read_text()
    api+='\nextern "C" void online_margin_debug(void*p,double*out){if(p)std::copy(static_cast<Online*>(p)->m.margin_debug.begin(),static_cast<Online*>(p)->m.margin_debug.end(),out);}\n'
    (out/'source/online_api.cpp').write_text(api)
    node=(parent/'source/mm_remani/src/dynamic_obstacle_tracker_node.cpp').read_text()
    node=replace(node,'  std::vector<Eigen::Vector3f> points;','  std::vector<Eigen::Vector3f> points;\n  std::vector<int> source_voxels;')
    node=replace(node,'    dynamic_points_pub_ = nh_.advertise', '    classified_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/sim/classified_cloud", 1);\n    dynamic_points_pub_ = nh_.advertise')
    node=replace(node,'    voxel.setInputCloud(input);','    voxel.setSaveLeafLayout(true);\n    voxel.setInputCloud(input);')
    node=replace(node,'    residual->reserve(downsampled->size());','    residual->reserve(downsampled->size());\n    std::vector<int> residual_source;')
    node=replace(node,'      if (!matches_static)\n        residual->push_back(candidate);',
                 '      if (!matches_static) {\n        residual->push_back(candidate);\n        residual_source.push_back(int(&point-downsampled->points.data()));\n      }')
    node=replace(node,'          cluster.points.push_back(value.cast<float>());',
                 '          cluster.points.push_back(value.cast<float>());\n          cluster.source_voxels.push_back(residual_source.at(index));')
    node=replace(node,'    if (publish_allowed)\n      publishDynamicPoints(message->header.stamp, clusters, tracked);',
                 '    if (publish_allowed) {\n      publishDynamicPoints(message->header.stamp, clusters, tracked);\n      publishClassified(message->header.stamp, *input, voxel, downsampled->size(), rotation, translation, clusters, tracked);\n    }')
    node=replace(node,'  void publishDynamicPoints(', (HERE/'classified_export.inc').read_text()+'\n  void publishDynamicPoints(')
    node=replace(node,'  ros::Publisher dynamic_points_pub_;','  ros::Publisher dynamic_points_pub_;\n  ros::Publisher classified_pub_;')
    (out/'source/dynamic_obstacle_tracker_node.cpp').write_text(node)
    for target,source,oldobj in [
        ('online_mppi','online_api.cpp','CMakeFiles/online_mppi.dir/online_api.cpp.o'),
        ('dynamic_obstacle_tracker_node','dynamic_obstacle_tracker_node.cpp','CMakeFiles/dynamic_obstacle_tracker_node.dir/builds/fixed_arm_01/source/mm_remani/src/dynamic_obstacle_tracker_node.cpp.o')]:
        cm=parent/'build/CMakeFiles'/ (target+'.dir')
        flags=(cm/'flags.make').read_text()
        inc=next(x.split('=',1)[1].strip() for x in flags.splitlines() if x.startswith('CXX_INCLUDES ='))
        obj=out/'build'/(source+'.o')
        subprocess.run(['/usr/bin/c++','-std=gnu++17','-O2','-DNDEBUG','-fPIC',*shlex.split(inc),'-c',str(out/'source'/source),'-o',str(obj)],check=True)
        link=shlex.split((cm/'link.txt').read_text())
        suffix='online_api.cpp.o' if target=='online_mppi' else 'dynamic_obstacle_tracker_node.cpp.o'
        matching=[x for x in link if x.endswith(suffix)];assert len(matching)==1
        link=[str(obj) if x==matching[0] else x for x in link]
        dest=out/'build'/('libonline_mppi.so' if target=='online_mppi' else target)
        link[link.index('-o')+1]=str(dest)
        subprocess.run(link,cwd=str(parent/'build'),check=True)
    m.update(controller_revision='native_mppi_v9_unified',classified_cloud=True,
        ablation_parent=str(base),mppi_library_sha256=sha(out/'build/libonline_mppi.so'),
        controller_changes=['V7/V8 candidate feasibility and recovery also enabled with zero dynamic tracks; no weight, geometry, speed or horizon changes',
          'static-only initially margin-deficient states: conservative body clearance >= 0.02m, monotonic clearance recovery, original full margin required at horizon end; no dynamic relaxation',
          'isolated detector publishes full-resolution same-frame measurements with voxel-source membership; unknowns retained',
          'native planner/controller binaries unchanged; abort and safety-speed authority retained'],
        isolated_source_sha256={p.name:sha(p) for p in (out/'source').iterdir()},
        builder_sha256=sha(Path(__file__)),exporter_sha256=sha(HERE/'classified_export.inc'))
    (out/'build_manifest.json').write_text(json.dumps(m,indent=2)+'\n')
    print(out,flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--base',type=Path,default=HERE/'builds/mppi_v8_09');p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();build(a.base,a.out)
