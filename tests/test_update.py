import sys
sys.path.append('../scripts')
import numpy as np

import unittest
import ekf
from states import States

class TestUpdate(unittest.TestCase):
    def test_all_zeros(self):
        p0 = np.zeros((3,1))
        q0 = np.zeros((3,1))
        v0 = np.zeros((3,1))
        ba0 = np.zeros((3,1))
        bg0 = np.zeros((3,1))
        cov0 = np.array([[1.0,1.0,1.0]]).T
        xHat = States(p0,q0,v0,ba0,bg0,cov0,cov0,cov0,cov0,cov0)
        Qt = np.zeros((7,7))
        zt = np.zeros((7,1))
        ht = np.zeros((7,1))

        pExpected = np.zeros((3,1))
        qExpected = np.zeros((3,1))
        vExpected = np.zeros((3,1))
        baExpected = np.zeros((3,1))
        bgExpected = np.zeros((3,1))
        statesExpected = States(pExpected,qExpected,vExpected,baExpected,bgExpected,cov0,cov0,cov0,cov0,cov0)
        
        Ct = ekf.get_jacobian_C()

        ekf.update(xHat,Qt,zt,ht,Ct)

        self.assert_states_equal(xHat,statesExpected)

    # def test_covariance_update(self):
    #     dt = 0.1
    #     Rt = np.array([[1.0,2.0,3.0],
    #                    [0.0,1.0,-1.0],
    #                    [1.0,1.0,-2.0]])
    #     A = np.array([[1.0,-1.0,1.0],
    #                   [2.0,2.0,0.0],
    #                   [0.0,1.0,1.0]])
    #     P = np.array([[2.0,1.0,0.0],
    #                   [0.0,2.0,-1.0],
    #                   [1.0,0.0,-1.0]])
    #     p0 = np.zeros((3,1))
    #     q0 = np.zeros((3,1))
    #     v0 = np.zeros((3,1))
    #     ba0 = np.zeros((3,1))
    #     bg0 = np.zeros((3,1))
    #     cov0 = np.array([[1.0,1.0,1.0]]).T
    #     xHat = States(p0,q0,v0,ba0,bg0,cov0,cov0,cov0,cov0,cov0)
    #     xHat.P = P
    #     xHat.A = A
    #     ekf.propagate(xHat,Rt,dt)
    #     expectedP = np.array([[5.0,6.0,2.0],
    #                           [-4.0,21.0,3.0],
    #                           [-2.0,7.0,-2.0]])
    #     self.assert_covariances_equal(xHat,expectedP)

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