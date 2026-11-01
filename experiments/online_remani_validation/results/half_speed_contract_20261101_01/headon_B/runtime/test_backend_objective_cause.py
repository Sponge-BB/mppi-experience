import unittest

import numpy as np

import audit_backend_objective_cause as audit


class Contracts(unittest.TestCase):
    def test_bilinear_linear_field_gradient(self):
        xx, yy = np.meshgrid(np.arange(4.), np.arange(4.))
        frame = dict(gx=0.,gy=0.,res=1.,nx=4,ny=4,field=.2+2.*xx+3.*yy)
        distance, gradient, unused = audit.bilinear(frame, [[.3,.7],[2.1,1.4]])
        np.testing.assert_allclose(distance,[2.9,8.6])
        np.testing.assert_allclose(gradient,[[2.,3.],[2.,3.]])

    def test_bilinear_domain_not_free(self):
        frame = dict(gx=0.,gy=0.,res=1.,nx=3,ny=3,field=np.ones((3,3)))
        with self.assertRaises(ValueError): audit.bilinear(frame, [[2.,1.]])

    def test_K8_includes_every_disk_and_endpoint(self):
        frame = dict(gx=-1.,gy=-1.,res=1.,nx=4,ny=4,field=np.full((4,4),.3),
            body=np.array([[0.,0.,.25],[0.,0.,.20]]),margin=.1,comfort=.12,weight=50000.)
        coeff=np.zeros((8,8));coeff[0,6]=.1
        candidate=dict(parts=[dict(gear=1,pieces=[dict(T=.5,coeffs=coeff.tolist())])])
        result=audit.projected_quadrature(candidate,frame)
        expected=.5*50000.*(.07**3+.02**3)
        self.assertAlmostEqual(expected,result['cost'],places=10)
        self.assertEqual(9,result['unsafe_disk_nodes'])
        self.assertEqual(9,result['zero_map_gradient_unsafe_nodes'])

    def test_five_point_component_FD(self):
        class Probe:
            def evaluate(self,x,context):
                return dict(costs={name:(i+1)*float(np.dot(x,x)) for i,name in enumerate(audit.COMPONENTS)})
        x=np.array([.2,-.3]);base=dict(free_indices=np.array([0,1]),
            gradients={name:(i+1)*2.*x for i,name in enumerate(audit.COMPONENTS)})
        result=audit.finite_difference(Probe(),None,x,base)
        self.assertTrue(result['primary_all_pass'])
        self.assertEqual(3e-6,result['primary_step'])

    def test_residual_never_named_collision(self):
        value=dict(gradients={name:np.array([1.,0.]) for name in audit.COMPONENTS},
                   free_indices=[0,1])
        attribution,gradients=audit.gradient_attribution(dict(g=[5.,1.]),value)
        np.testing.assert_array_equal(gradients['unreplayed_native_remainder'],[1.,1.])
        self.assertNotIn('native_collision',gradients)
        self.assertAlmostEqual(26.,attribution['all_free']['total_norm']**2)


if __name__=='__main__': unittest.main()
