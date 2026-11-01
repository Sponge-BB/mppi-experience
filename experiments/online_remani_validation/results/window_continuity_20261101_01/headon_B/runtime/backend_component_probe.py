#!/usr/bin/env python3
"""Read-only offline native MINCO component probe, not a native 3-D replay.

The original wheel quadrature (including its dense branch), MinSnap reverse
pass, time mapping, and additive projected cost are compiled from attested
source. Native obstacle/joint costs are explicitly stubbed out. Callers may
subtract these known components from a recorded f/g, but must call the result
``unreplayed_native_remainder``, not a replayed collision objective.
"""
import ctypes
import hashlib
import json
from pathlib import Path
import subprocess

import numpy as np

from audit_backend_wheel_gradient import function, native_parameters
from audit_delivery_fullcurve import frozen_frame
from audit_delivery_iterates import canonical_records, matrix, vector_hash

BASE = Path(__file__).resolve().parent
WORKSPACE = BASE.parents[1]
DEFAULT_BUILD = BASE/'builds/feasibility_stop_20261026_01'
COMPONENTS = ('smooth', 'time', 'wheel', 'projected_static')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def extract(text, needle, template=False):
    if text.count(needle) != 1:
        raise ValueError('Expected unique source block: '+needle)
    begin = text.index(needle)
    if template:
        begin = text.rfind('template', 0, begin)
    opening = text.index('{', begin)
    depth = 0
    for end in range(opening, len(text)):
        depth += (text[end] == '{') - (text[end] == '}')
        if not depth:
            return text[begin:end+1]
    raise ValueError('Unclosed source block')


def source_material(build=DEFAULT_BUILD):
    build = Path(build).resolve()
    manifest = json.loads((build/'build_manifest.json').read_text())
    sources = {}
    for name in ('poly_traj_optimizer.cpp', 'coupling_collision.hpp',
                 'shared_collision.hpp', 'coupling_geometry_ros.hpp',
                 'coupling_native_integration.hpp'):
        path = build/'source'/name
        if sha(path) != manifest['isolated_source_sha256'][name]:
            raise ValueError('Unattested isolated source: '+name)
        sources[name] = path.read_text()
    for name in ('poly_traj_utils.hpp', 'root_finder.hpp'):
        path = WORKSPACE/'src/mm_remani_planner/traj_utils/include/traj_utils'/name
        if sha(path) != manifest['production_sources_sha256'][str(path.relative_to(WORKSPACE))]:
            raise ValueError('Unattested production header: '+name)
        sources[name] = path.read_text()
    state_header=WORKSPACE/'src/mm_remani_planner/mm_config/include/mm_config/mm_config.hpp'
    if sha(state_header)!=manifest['production_sources_sha256'][str(state_header.relative_to(WORKSPACE))]:
        raise ValueError('Unattested MMState declaration')
    sources['mm_config.hpp']=state_header.read_text()
    headers = {name:hashlib.sha256(text.encode()).hexdigest() for name,text in sources.items()}
    return sources, dict(build=str(build), build_manifest_sha256=sha(build/'build_manifest.json'),
                        sources_sha256=headers)


CPP_SHELL = r'''
#include <Eigen/Dense>
#include <string>
#include <vector>
#include <memory>
#include <stdexcept>
#include "backend_component_probe_minco.hpp"
#include "coupling_collision.hpp"
namespace failure_probe {
enum { BASE_FEAS, INTEGRAL, SMOOTH };
struct Timer { explicit Timer(int) {} };
}
class PolyTrajOptimizer {
public:
 int traj_dim_=8,manipulator_dof_=6,dense_sample_resolution_=20;
 bool mani_hold_enabled_=false; double wei_mani_hold_=0.;
 Eigen::VectorXd mani_hold_reference_=Eigen::VectorXd::Zero(6);
 Eigen::Matrix2d B_h_;
 std::vector<int> singul_container_;
 std::vector<poly_traj::MinSnapOpt<8>> SnapOpt_container_;
 struct Cps { Eigen::MatrixXd points; }; std::vector<Cps> cps_container_;
 double wei_feas_,max_vel_,non_singul_v_,mobile_base_wheel_base_,
   mobile_base_wheel_radius_,max_wheel_omega_,max_wheel_alpha_,wei_time_;
 bool smoothedL1(const double&,const double&,double&,double&);
 bool smoothedLog(const double&,const double&,double&,double&);
 bool feasibilityGradCostCar(const Eigen::Vector2d&,const Eigen::Vector2d&,
   const Eigen::Vector2d&,int,Eigen::Vector2d&,Eigen::Vector2d&,Eigen::Vector2d&,double&,bool);
 template<typename V> void addPVAJGradCost2CT(int,V&,Eigen::VectorXd&,const int&);
 template<typename V> void VirtualT2RealT(const V&,Eigen::VectorXd&);
 template<typename V,typename G> void VirtualTGradCost(const Eigen::VectorXd&,const V&,
   const Eigen::VectorXd&,G&,double&);
 bool obstacleGradCostforMM(int,const Eigen::VectorXd&,const Eigen::VectorXd&,int,
   Eigen::VectorXd&,Eigen::VectorXd&,double&,double&,double&) {return false;}
 bool feasibilityGradCostJoint(const Eigen::VectorXd&,const Eigen::VectorXd&,
   const Eigen::VectorXd&,Eigen::VectorXd&,Eigen::VectorXd&,Eigen::VectorXd&,double&)
   {return false;}
};
'''


CPP_GEOMETRY_SHIM = r'''
namespace coupling_geometry {
using Snapshot=shared_collision::Snapshot;
using Evaluation=coupling_collision::Evaluation;
inline bool enabled(){return true;}
static double probe_epoch=0.;
class Scope {
public:
 bool valid()const {auto f=shared_collision::snapshot();return f&&shared_collision::fresh(*f,probe_epoch);}
 std::shared_ptr<const Snapshot> snapshot()const{return shared_collision::snapshot();}
 double now()const{return probe_epoch;}
};
inline Evaluation staticProjectedPose(const Snapshot& f,const Eigen::Vector3d& q){
 coupling_collision::Options op;op.include_dynamic=false;
 return coupling_collision::evaluate(f,q.x(),q.y(),q.z(),0.,op);
}
}
namespace delivery_diag {
template<class... T> void rememberSolve(const T&...){}
// The captured valid static curves never enter geometryError; fail loudly if they do.
template<class A,class B,class C,class E,class F,class G,class H,class I,class J,class K,class L,class M,class N,class O,class P>
void geometryError(const A&,const B&,const C&,std::pair<std::shared_ptr<const shared_collision::Snapshot>,double>,
 const E&,const F&,const G&,const H&,const I&,const J&,const K&,const L&,const M&,const N&,const O&){
 throw std::runtime_error("invalid geometry probe sample");
}
}
namespace domain_step {struct RejectedTrial:std::runtime_error {
template<class... T> explicit RejectedTrial(T...):std::runtime_error("outside projected map"){} };}
namespace coupling_native {
constexpr int geometry_quadrature_intervals=8;
inline bool domainRecoveryEnabled(){return false;}}
'''


CPP_EVALUATE = r'''
static thread_local std::string probe_error;
extern "C" const char* backend_component_probe_error(){return probe_error.c_str();}
extern "C" int backend_component_evaluate(int blocks,const int* pieces,const int* gears,
 const double* ini,const double* fin,const double* params,int snapshot_n,const double* snapshot,
 double epoch,int nx,const double* xv,double* costs,double* gradients,double* coeff,double* times_out){
 try {
  if(blocks<1||blocks>30)throw std::invalid_argument("invalid block count");
  int total=0,np=0;for(int t=0;t<blocks;++t){
    if(pieces[t]<1||pieces[t]>1000||(gears[t]!=1&&gears[t]!=-1))throw std::invalid_argument("bad pieces/gear");
    total+=pieces[t];np+=8*(pieces[t]-1);
  }
  if(nx!=np+total+9*(blocks-1))throw std::invalid_argument("wrong x dimension");
  Eigen::Map<const Eigen::VectorXd>x(xv,nx);
  if(!x.allFinite())throw std::invalid_argument("nonfinite x");
  if(!shared_collision::bind(snapshot,snapshot_n))throw std::invalid_argument("invalid frozen map transport");
  if(!shared_collision::snapshot()->obstacles.empty())throw std::invalid_argument("static probe cannot replay dynamic actors");
  coupling_geometry::probe_epoch=epoch;
  PolyTrajOptimizer opt;opt.SnapOpt_container_.resize(blocks);opt.cps_container_.resize(blocks);
  opt.B_h_<<0.,-1.,1.,0.;opt.singul_container_.assign(gears,gears+blocks);
  opt.wei_feas_=params[0];opt.max_vel_=params[1];opt.non_singul_v_=params[2];
  opt.mobile_base_wheel_base_=params[3];opt.mobile_base_wheel_radius_=params[4];
  opt.max_wheel_omega_=params[5];opt.max_wheel_alpha_=params[6];
  const double time_weight=params[7];const int K=int(params[8]);
  opt.dense_sample_resolution_=int(params[9]);const bool opt_gear=params[10]!=0.;
  if(K<1||opt.dense_sample_resolution_<2)throw std::invalid_argument("bad quadrature resolution");
  std::vector<Eigen::MatrixXd>P(blocks),I(blocks),F(blocks);
  std::vector<Eigen::VectorXd>T(blocks),vt(blocks);
  int offset=0;
  for(int t=0;t<blocks;++t){P[t]=Eigen::Map<const Eigen::MatrixXd>(xv+offset,8,pieces[t]-1);offset+=8*(pieces[t]-1);}
  for(int t=0;t<blocks;++t){vt[t]=x.segment(offset,pieces[t]);T[t].resize(pieces[t]);
    opt.VirtualT2RealT(vt[t],T[t]);offset+=pieces[t];}
  const int gear_offset=offset,angle_offset=offset+8*(blocks-1);
  int output_piece=0;
  for(int t=0;t<blocks;++t){
    I[t]=Eigen::Map<const Eigen::Matrix<double,8,4>>(ini+32*t);
    F[t]=Eigen::Map<const Eigen::Matrix<double,8,4>>(fin+32*t);
    if(t>0){double a=x[angle_offset+t-1];I[t].col(0)=x.segment(gear_offset+8*(t-1),8);
      I[t].col(1).head(2)=Eigen::Vector2d(-opt.non_singul_v_*cos(a),-opt.non_singul_v_*sin(a));}
    if(t<blocks-1){double a=x[angle_offset+t];F[t].col(0)=x.segment(gear_offset+8*t,8);
      F[t].col(1).head(2)=Eigen::Vector2d(opt.non_singul_v_*cos(a),opt.non_singul_v_*sin(a));}
    opt.SnapOpt_container_[t].reset(pieces[t]);
    opt.SnapOpt_container_[t].generate(P[t],T[t],I[t],F[t]);
    opt.cps_container_[t].points.resize(8,pieces[t]*K+1);
    for(int p=0;p<pieces[t];++p){times_out[output_piece]=T[t][p];
      for(int r=0;r<8;++r)for(int c=0;c<8;++c)
        coeff[output_piece*64+r*8+c]=opt.SnapOpt_container_[t].get_b()(p*8+r,c);
      ++output_piece;
    }
  }
  for(int component=0;component<4;++component){
    Eigen::Map<Eigen::VectorXd>g(gradients+component*nx,nx);g.setZero();
    std::vector<Eigen::VectorXd>gdT(blocks);costs[component]=0.;
    for(int t=0;t<blocks;++t){gdT[t]=Eigen::VectorXd::Zero(pieces[t]);
      opt.SnapOpt_container_[t].get_gdC().setZero();
      if(component==0){double c=0.;opt.SnapOpt_container_[t].initGradCost(gdT[t],c);costs[component]+=c;}
      if(component==2){Eigen::VectorXd c(6);opt.addPVAJGradCost2CT(t,gdT[t],c,K);costs[component]+=c[3];}
    }
    if(component==3)costs[component]=coupling_native::addGeometry(opt.SnapOpt_container_,T,
      opt.singul_container_,gdT,8);
    opt.wei_time_=component==1?time_weight:0.;
    int po=0,to=np;
    for(int t=0;t<blocks;++t){
      Eigen::Map<Eigen::MatrixXd>gp(g.data()+po,8,pieces[t]-1);po+=8*(pieces[t]-1);
      Eigen::MatrixXd gi,gf;opt.SnapOpt_container_[t].getGrad2TP(gdT[t],gp,gi,gf);
      if(opt_gear){
       if(t>0){double a=x[angle_offset+t-1];g.segment(gear_offset+8*(t-1),8)+=gi.col(0);
        g[angle_offset+t-1]+=gi.topRows(2).col(1).dot(Eigen::Vector2d(opt.non_singul_v_*sin(a),-opt.non_singul_v_*cos(a)));}
       if(t<blocks-1){double a=x[angle_offset+t];g.segment(gear_offset+8*t,8)+=gf.col(0);
        g[angle_offset+t]+=gf.topRows(2).col(1).dot(Eigen::Vector2d(-opt.non_singul_v_*sin(a),opt.non_singul_v_*cos(a)));}
      }
      Eigen::Map<Eigen::VectorXd>gt(g.data()+to,pieces[t]);to+=pieces[t];double ct=0.;
      opt.VirtualTGradCost(T[t],vt[t],gdT[t],gt,ct);costs[component]+=ct;
      gp.bottomRows(6).setZero();
    }
    for(int t=0;t<blocks-1;++t)g.segment(gear_offset+8*t+2,6).setZero();
    if(!std::isfinite(costs[component])||!g.allFinite())throw std::runtime_error("nonfinite component result");
  }
  probe_error.clear();return 0;
 }catch(const std::exception& e){probe_error=e.what();return -1;}
}
'''


def cpp_probe(sources):
    text = sources['poly_traj_optimizer.cpp']
    definitions = [function(text,name) for name in ('smoothedL1','smoothedLog','feasibilityGradCostCar')]
    definitions += [extract(text, 'void PolyTrajOptimizer::'+name+'(', True)
                    for name in ('VirtualT2RealT','VirtualTGradCost','addPVAJGradCost2CT')]
    geometry = sources['coupling_geometry_ros.hpp']
    sample = extract(geometry, 'struct ObjectiveSample {')+';'
    projection = extract(geometry, 'inline ObjectiveSample projectionCost(')
    add = extract(sources['coupling_native_integration.hpp'], 'double addGeometry(', True)
    # geometryError has an undeducible braced argument in the original logger;
    # replace only that diagnostic call with a loud exception. It is reached
    # only on invalid samples and has no finite-domain objective effect.
    logger = ('delivery_diag::geometryError(opts,real_times,signs,{frame,epoch},block,\n'
              '            int(piece),sample,prefix,s,beta0,beta1,beta2,p,v,a);')
    if add.count(logger)!=1:
        raise ValueError('unknown invalid geometry diagnostic call')
    add = add.replace(logger,'throw std::runtime_error("invalid projected geometry sample");')
    mmstate=extract(sources['mm_config.hpp'],'struct MMState{')+';'
    shell=CPP_SHELL.replace('#include "backend_component_probe_minco.hpp"',
          'namespace remani_planner {\n'+mmstate+'\n}\n#include "backend_component_probe_minco.hpp"')
    if 'constexpr int geometry_quadrature_intervals=8;' not in sources['coupling_native_integration.hpp']:
        raise ValueError('Unknown projected quadrature constant')
    return (shell+'\n'+'\n'.join(definitions)+CPP_GEOMETRY_SHIM+
            '\nnamespace coupling_geometry {\n'+sample+'\n'+projection+'\n}\n'+
            '\nnamespace coupling_native {\n'+add+'\n}\n'+CPP_EVALUATE)


def build_probe(output_dir, build=DEFAULT_BUILD):
    output = Path(output_dir).resolve()
    output.mkdir(parents=True, exist_ok=False)
    sources, provenance = source_material(build)
    minco=sources['poly_traj_utils.hpp']
    remove='#include "mm_config/mm_config.hpp"'
    if minco.count(remove)!=1:
        raise ValueError('unknown MinSnap unused config include')
    (output/'backend_component_probe_minco.hpp').write_text(minco.replace(remove,''))
    for name in ('shared_collision.hpp','coupling_collision.hpp'):
        (output/name).write_text(sources[name])
    cpp=output/'backend_component_probe.cpp';cpp.write_text(cpp_probe(sources))
    library=output/'backend_component_probe.so'
    command=['g++','-std=c++14','-O2','-DNDEBUG','-shared','-fPIC',
             '-I/usr/include/eigen3','-I'+str(WORKSPACE/'src/mm_remani_planner/traj_utils/include'),
             str(cpp),'-o',str(library)]
    completed=subprocess.run(command,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
    (output/'compile.log').write_text(completed.stdout)
    provenance.update(command=command,returncode=completed.returncode,
        native_obstacle_stubbed_false=True,native_joint_stubbed_false=True,
        arm_hold_disabled=True,native_3d_replayed=False,
        geometry_invalid_logger='throws rather than logging; finite-domain math unchanged',
        generated_cpp_sha256=sha(cpp),probe_python_sha256=sha(__file__))
    if completed.returncode==0:
        provenance['library_sha256']=sha(library)
    (output/'manifest.json').write_text(json.dumps(provenance,indent=2)+'\n')
    if completed.returncode:
        raise RuntimeError('Probe compilation failed: '+str(output/'compile.log'))
    return library,provenance


class Context:
    @classmethod
    def from_trial(cls, trial, phase='delivery_nominal_060'):
        self=cls();self.trial=Path(trial).resolve();self.phase=phase
        records,path=canonical_records(self.trial)
        captures=[r for r in records if r.get('kind')=='capture']
        summaries=[r for r in records if r.get('phase')==phase and r.get('kind')=='solve_summary']
        if len(captures)!=1 or len(summaries)!=1:
            raise ValueError('Ambiguous captured request/phase')
        capture,summary=captures[0],summaries[0]
        self.records=[r for r in records if r.get('phase')==phase and r.get('kind')=='canonical_candidate']
        if not self.records:raise ValueError('No canonical candidates')
        self.gears=np.asarray(capture['directions'],np.int32)
        self.pieces=np.asarray([len(p['times']) for p in self.records[0]['canonical_parts']],np.int32)
        self.ini=np.concatenate([matrix(s).ravel(order='F') for s in summary['effective_initial_states']])
        self.fin=np.concatenate([matrix(s).ravel(order='F') for s in summary['effective_final_states']])
        wheel,_,wheelproof=native_parameters(self.trial)
        loaded=json.loads((self.trial/'effective_real_params.json').read_text())
        prefix='/remani_planner_node/optimization/'
        k=wheelproof['quadrature_intervals'];dense=wheelproof['dense_resolution']
        self.params=np.asarray(wheel+[loaded[prefix+'weight_time'],k,dense,
                                      int(loaded[prefix+'mobile_base_opt_gear'])],dtype=float)
        frame,proof=frozen_frame(self.trial/'delivery_diagnostics',capture['frame'])
        if len(frame['obstacles']):raise ValueError('Probe restricted to static captures')
        self.epoch=float(capture['frame']['now'])
        self.snapshot=np.asarray([1,frame['sequence'],frame['stamp'],frame['nx'],frame['ny'],
             frame['gx'],frame['gy'],frame['res'],frame['margin'],frame['comfort'],frame['weight'],
             len(frame['body']),0,frame['max_age'],frame['max_prediction'],int(frame['healthy'])]+
             frame['body'].ravel().tolist()+frame['field'].ravel().tolist(),dtype=float)
        self.nx=len(self.records[0]['x'])
        np_count=int(sum(8*(self.pieces-1)));total=int(sum(self.pieces))
        free=[];fixed=[]
        for start in list(range(0,np_count,8))+list(range(np_count+total,np_count+total+8*(len(self.pieces)-1),8)):
            free.extend([start,start+1]);fixed.extend(range(start+2,start+8))
        free.extend(range(np_count,np_count+total));free.extend(range(np_count+total+8*(len(self.pieces)-1),self.nx))
        self.free_indices=np.asarray(sorted(free),dtype=int);self.fixed_indices=np.asarray(sorted(fixed),dtype=int)
        self.coord_groups=dict(point_xy=[i for i in range(np_count) if i%8<2],
            time=list(range(np_count,np_count+total)),
            gear_xy=[np_count+total+8*t+d for t in range(len(self.pieces)-1) for d in (0,1)],
            gear_angle=list(range(np_count+total+8*(len(self.pieces)-1),self.nx)))
        self.metadata=dict(trial=str(self.trial),phase=phase,canonical_file=str(path),canonical_sha256=sha(path),
            parameter_sha256=sha(self.trial/'effective_real_params.json'),wheel_provenance=wheelproof,
            frozen_snapshot_proof=proof,unreplayed_native_remainder_is_not_collision_replay=True,
            fixed_arm_coordinates_are_eliminated=True)
        if len(self.ini)!=32*len(self.pieces) or len(self.fin)!=len(self.ini):raise ValueError('bad boundaries')
        if set(self.free_indices)|set(self.fixed_indices)!=set(range(self.nx)):raise ValueError('incomplete x coordinate partition')
        return self


class Probe:
    def __init__(self, library):
        self.library=ctypes.CDLL(str(Path(library).resolve()))
        self.call=self.library.backend_component_evaluate
        pointer=ctypes.POINTER(ctypes.c_double);integer=ctypes.POINTER(ctypes.c_int)
        self.call.argtypes=(ctypes.c_int,integer,integer,pointer,pointer,pointer,ctypes.c_int,pointer,
                           ctypes.c_double,ctypes.c_int,pointer,pointer,pointer,pointer,pointer)
        self.call.restype=ctypes.c_int
        self.library.backend_component_probe_error.restype=ctypes.c_char_p

    def evaluate(self, x, context):
        x=np.ascontiguousarray(x,dtype=float)
        if x.shape!=(context.nx,) or not np.all(np.isfinite(x)):raise ValueError('bad x')
        pointer=ctypes.POINTER(ctypes.c_double);integer=ctypes.POINTER(ctypes.c_int)
        d=lambda a:np.ascontiguousarray(a,dtype=np.float64).ctypes.data_as(pointer)
        costs=np.zeros(4);gradients=np.zeros((4,context.nx));total=int(sum(context.pieces))
        coeff=np.zeros((total,8,8));times=np.zeros(total)
        result=self.call(len(context.pieces),context.pieces.ctypes.data_as(integer),context.gears.ctypes.data_as(integer),
          d(context.ini),d(context.fin),d(context.params),len(context.snapshot),d(context.snapshot),context.epoch,
          context.nx,d(x),d(costs),d(gradients),d(coeff),d(times))
        if result:raise ValueError(self.library.backend_component_probe_error().decode())
        bp=[];tp=[];offset=0
        for count in context.pieces:
            bp.append(coeff[offset:offset+count].reshape(count*8,8).copy())
            tp.append(times[offset:offset+count].copy());offset+=count
        return dict(costs=dict(zip(COMPONENTS,costs.tolist())),
          gradients={name:gradients[i].copy() for i,name in enumerate(COMPONENTS)},
          b_parts=bp,times_parts=tp,free_indices=context.free_indices.copy(),metadata=context.metadata)


def compare_canonical(value, record):
    checks=[]
    for b,t,part in zip(value['b_parts'],value['times_parts'],record['canonical_parts']):
        expected_b=np.asarray(part['b_rows_ascending_power'],float);expected_t=np.asarray(part['times'],float)
        checks.append(dict(b_max_abs_error=float(np.max(abs(b-expected_b))),
          b_max_normalized_error=float(np.max(abs(b-expected_b)/np.maximum(1.,abs(expected_b)))),
          times_max_abs_error=float(np.max(abs(t-expected_t))),
          b_bitwise_equal=bool(np.array_equal(b.view(np.uint64),expected_b.view(np.uint64))),
          times_bitwise_equal=bool(np.array_equal(t.view(np.uint64),expected_t.view(np.uint64)))))
    return dict(x_hash=vector_hash(record['x']),parts=checks,
                verified=all(c['b_max_normalized_error']<1e-10 and c['times_max_abs_error']<1e-12 for c in checks))


def finite_difference(probe, context, record, multipliers=(1e-5,3e-6,1e-6)):
    x=np.asarray(record['x'],float);base=probe.evaluate(x,context);rows=[]
    for index in context.free_indices:
        row=dict(index=int(index),analytic={name:float(base['gradients'][name][index]) for name in COMPONENTS},steps=[])
        for multiplier in multipliers:
            h=float(multiplier*max(1.,abs(x[index])));values=[]
            for k in (-2.,-1.,1.,2.):
                shifted=x.copy();shifted[index]+=k*h;values.append(probe.evaluate(shifted,context)['costs'])
            numerical={name:(8*(values[2][name]-values[1][name])-(values[3][name]-values[0][name]))/(12*h)
                       for name in COMPONENTS}
            errors={name:abs(row['analytic'][name]-numerical[name])/max(1.,abs(row['analytic'][name]),abs(numerical[name]))
                    for name in COMPONENTS}
            row['steps'].append(dict(multiplier=multiplier,h=h,five_point=numerical,normalized_error=errors))
        rows.append(row)
    return dict(tested_coordinates=context.free_indices.tolist(),fixed_arm_coordinates_excluded=context.fixed_indices.tolist(),
                primary_step_index=1,tolerance=1e-4,coordinates=rows)
