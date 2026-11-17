"""Compile the transformed native function and compare its collision policy."""
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

import numpy as np

from active_arm_model import PiperArmModel, LINK_POINTS, DH_A, DH_D, DH_ALPHA, DH_OFFSET
from active_arm_model_transform import transform_mm_config


BASELINE = Path(__file__).parent/'builds/short_prefix_rejoin_20261101_01/source/mm_config.cpp'


class NativeSelfCollisionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not shutil.which('g++'): raise unittest.SkipTest('g++ is unavailable')
        cls.directory = tempfile.TemporaryDirectory(prefix='active_arm_self_')
        directory = Path(cls.directory.name)
        transformed = transform_mm_config(BASELINE.read_text())
        start = transformed.index('bool MMConfig::checkManiManiCollision(')
        end = transformed.index('\nbool MMConfig::checkManicollision(',start)
        function = transformed[start:end]
        declarations = '\n'.join('const double '+name+'[6] = {'+','.join(format(float(v),'.17g') for v in value)+'};'
                                 for name,value in [('aa',DH_A),('dd',DH_D),('alpha',DH_ALPHA),('offset',DH_OFFSET)])
        initializers = '\n'.join('manipulator_link_pts_.emplace_back(4,'+str(len(points))+');\n'+
            '\n'.join('manipulator_link_pts_.back().col('+str(j)+') << '+','.join(str(v) for v in [*point,1.])+';'
                      for j,point in enumerate(points)) for points in LINK_POINTS)
        source = '''#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>
class MMConfig {
 public:
 bool useFastArmer_ = false; int manipulator_dof_ = 6;
 double manipulator_thickness_ = .034, self_safe_margin_ = .1;
 std::vector<Eigen::Matrix<double,4,Eigen::Dynamic>> manipulator_link_pts_;
 MMConfig() { INITIALIZERS }
 void getJointTrans(Eigen::VectorXd q, std::vector<Eigen::Matrix4d>& result,
                    std::vector<Eigen::Matrix4d>& unused) {
  DECLARATIONS
  for (int i=0;i<6;++i) {
   double c=std::cos(q(i)+offset[i]),s=std::sin(q(i)+offset[i]);
   double ca=std::cos(alpha[i]),sa=std::sin(alpha[i]);
   Eigen::Matrix4d t;
   t << c,-s*ca,s*sa,aa[i]*c, s,c*ca,-c*sa,aa[i]*s,
        0,sa,ca,dd[i], 0,0,0,1;
   result.push_back(t);
  }
 }
 bool checkManiManiCollision(Eigen::VectorXd q,bool safe,double& min_dist);
};
FUNCTION
int main() {
 MMConfig model; Eigen::VectorXd q(6); std::cout << std::setprecision(17);
 while(std::cin >> q(0)) {
  for(int i=1;i<6;++i) std::cin >> q(i);
  double gap; bool hard=model.checkManiManiCollision(q,false,gap);
  std::cout << hard << " " << gap << " " << model.checkManiManiCollision(q,true,gap) << "\\n";
 }
}
'''.replace('INITIALIZERS',initializers).replace('DECLARATIONS',declarations).replace('FUNCTION',function)
        cpp=directory/'self.cpp'; cpp.write_text(source); cls.binary=directory/'self'
        subprocess.run(['g++','-std=c++14','-O1','-I/usr/include/eigen3',str(cpp),'-o',str(cls.binary)],check=True,capture_output=True,text=True)

    @classmethod
    def tearDownClass(cls):
        if hasattr(cls,'directory'): cls.directory.cleanup()

    def test_compiled_native_policy_matches_python_random_configurations(self):
        model=PiperArmModel.from_defaults()
        qs=np.r_[np.random.default_rng(84).uniform(model.lower,model.upper,(100,6)),
                 np.zeros((1,6)),model.q_nominal[None,:]]
        text='\n'.join(' '.join(format(float(v),'.17g') for v in q) for q in qs)+'\n'
        result=subprocess.run([str(self.binary)],input=text,capture_output=True,text=True,check=True)
        rows=np.loadtxt(result.stdout.splitlines())
        gaps=np.array([model.check(q)['min_self_gap'] for q in qs])
        np.testing.assert_allclose(rows[:,1]-.068,gaps,atol=1e-14)
        np.testing.assert_array_equal(rows[:,0],gaps<0.)
        np.testing.assert_array_equal(rows[:,2],gaps<.1)

    def test_transform_fails_closed_on_repeated_or_changed_source(self):
        source=BASELINE.read_text()
        result=transform_mm_config(source)
        self.assertEqual(result.count('ACTIVE_ARM_NATIVE20_SELF'),1)
        self.assertEqual(result.count('if(pt_on_link(2) - manipulator_thickness_ < ground_safe_dis_)'),2)
        with self.assertRaises(ValueError): transform_mm_config(result)
        with self.assertRaises(ValueError): transform_mm_config('unexpected source')


if __name__=='__main__': unittest.main()
