import sys
sys.path.append('/home/matt/px4_ws/src/boat_estimator/scripts')
import numpy as np

import unittest
import ekf
from states import States

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
        xHat = States(p0,q0,v0,ba0,bg0,cov0,cov0,cov0,cov0,cov0)
        xHat.dp = np.array([[1.2,-0.3,0.0]]).T
        xHat.dq = np.array([[-0.1,-2.1,1.0]]).T
        xHat.dv = np.array([[3.4,1.1,2.0]]).T
        xHat.dba = np.array([[4.3,0.0,-0.0]]).T
        xHat.dbg = np.array([[3.2,3.0,0.22]]).T
        pExpected = np.array([[0.22,-0.23,4.0]]).T
        qExpected = np.array([[-0.01,-2.41,3.1]]).T
        vExpected = np.array([[2.54,4.21,2.4]]).T
        baExpected = np.array([[0.53,2.1,-1.0]]).T
        bgExpected = np.array([[8.72,2.4,-1.978]]).T
        statesExpected = States(pExpected,qExpected,vExpected,baExpected,bgExpected,cov0,cov0,cov0,cov0,cov0)
        ekf.propagate(xHat,Rt,dt)
        self.assert_states_equal(xHat,statesExpected)

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
        xHat = States(p0,q0,v0,ba0,bg0,cov0,cov0,cov0,cov0,cov0)
        xHat.P = P
        xHat.At = At
        ekf.propagate(xHat,Rt,dt)
        expectedP = np.array([[5.0,6.0,2.0],
                              [-4.0,21.0,3.0],
                              [-2.0,7.0,-2.0]])
        self.assert_covariances_equal(xHat,expectedP)

    def assert_states_equal(self,xHat,statesExpected):
        for i in range(3):
            self.assertAlmostEqual(xHat.p.item(i),statesExpected.p.item(i))
            self.assertAlmostEqual(xHat.q.item(i),statesExpected.q.item(i))
            self.assertAlmostEqual(xHat.v.item(i),statesExpected.v.item(i))
            self.assertAlmostEqual(xHat.ba.item(i),statesExpected.ba.item(i))
            self.assertAlmostEqual(xHat.bg.item(i),statesExpected.bg.item(i))

    def assert_covariances_equal(self,xHat,expectedP):
        for i in range(3):
            for j in range(3):
                self.assertAlmostEqual(xHat.P[i][j],expectedP[i][j])


if __name__ == '__main__':
    unittest.main()