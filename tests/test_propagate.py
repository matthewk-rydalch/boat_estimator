import sys
sys.path.append('/home/matt/px4_ws/src/boat_estimator/scripts')
import numpy as np

import unittest
import ekf
from dynamic_model import DynamicModel
from states_covariance import StatesCovariance

class TestPropagation(unittest.TestCase):
    def test_state_propagation(self):
        dt = 0.1
        Rt = np.zeros((15,15))
        p0 = np.array([[0.1,-0.2,4.0]]).T
        q0 = np.array([[-0.0,-2.2,3.0]]).T
        v0 = np.array([[2.2,4.1,2.2]]).T
        ba0 = np.array([[0.1,2.1,-1.0]]).T
        bg0 = np.array([[8.4,2.1,-2.0]]).T
        cov0 = np.array([[1.0,1.0,1.0]]).T
        beleif = StatesCovariance(p0,q0,v0,ba0,bg0,cov0,cov0,cov0,cov0,cov0)
        ft = DynamicModel()
        ft.dp = np.array([[1.2,-0.3,0.0]]).T
        ft.dq = np.array([[-0.1,-2.1,1.0]]).T
        ft.dv = np.array([[3.4,1.1,2.0]]).T
        ft.dba = np.array([[4.3,0.0,-0.0]]).T
        ft.dbg = np.array([[3.2,3.0,0.22]]).T
        omega = np.zeros((3,1))
        At = ekf.update_Jacobian_A(beleif,omega)
        pExpected = np.array([[0.22,-0.23,4.0]]).T
        qExpected = np.array([[-0.01,-2.41,3.1]]).T
        vExpected = np.array([[2.54,4.21,2.4]]).T
        baExpected = np.array([[0.53,2.1,-1.0]]).T
        bgExpected = np.array([[8.72,2.4,-1.978]]).T
        beleifExpected = StatesCovariance(pExpected,qExpected,vExpected,baExpected,bgExpected,cov0,cov0,cov0,cov0,cov0)
        ekf.propagate(beleif,Rt,ft,At,dt)
        self.assert_states_equal(beleif,beleifExpected)

    def test_covariance_propagation(self):
        dt = 0.1
        Rt = np.array([[1.0,2.0,3.0],
                       [0.0,1.0,-1.0],
                       [1.0,1.0,-2.0]])
        At = np.array([[1.0,-1.0,1.0],
                      [2.0,2.0,0.0],
                      [0.0,1.0,1.0]])
        P = np.array([[2.0,1.0,0.0],
                      [0.0,2.0,-1.0],
                      [1.0,0.0,-1.0]])
        p0 = np.zeros((3,1))
        q0 = np.zeros((3,1))
        v0 = np.zeros((3,1))
        ba0 = np.zeros((3,1))
        bg0 = np.zeros((3,1))
        cov0 = np.array([[1.0,1.0,1.0]]).T
        beleif = StatesCovariance(p0,q0,v0,ba0,bg0,cov0,cov0,cov0,cov0,cov0)
        beleif.P = P
        ft = DynamicModel()
        ekf.propagate(beleif,Rt,ft,At,dt)
        expectedP = np.array([[5.0,6.0,2.0],
                              [-4.0,21.0,3.0],
                              [-2.0,7.0,-2.0]])
        self.assert_covariances_equal(beleif,expectedP)

    def assert_states_equal(self,beleif,beleifExpected):
        for i in range(3):
            self.assertAlmostEqual(beleif.p.item(i),beleifExpected.p.item(i))
            self.assertAlmostEqual(beleif.q.item(i),beleifExpected.q.item(i))
            self.assertAlmostEqual(beleif.v.item(i),beleifExpected.v.item(i))
            self.assertAlmostEqual(beleif.ba.item(i),beleifExpected.ba.item(i))
            self.assertAlmostEqual(beleif.bg.item(i),beleifExpected.bg.item(i))

    def assert_covariances_equal(self,beleif,expectedP):
        for i in range(3):
            for j in range(3):
                self.assertAlmostEqual(beleif.P[i][j],expectedP[i][j])


if __name__ == '__main__':
    unittest.main()