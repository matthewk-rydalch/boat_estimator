import sys
sys.path.append('/home/matt/px4_ws/src/boat_estimator/scripts')
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
        Qt = np.identity(7)
        zt = np.zeros((7,1))
        ht = ekf.update_measurement_model(xHat)

        pExpected = np.zeros((3,1))
        qExpected = np.zeros((3,1))
        vExpected = np.zeros((3,1))
        baExpected = np.zeros((3,1))
        bgExpected = np.zeros((3,1))
        statesExpected = States(pExpected,qExpected,vExpected,baExpected,bgExpected,cov0,cov0,cov0,cov0,cov0)

        expectedP = np.diag([0.5,0.5,0.5,1.0,1.0,0.5,0.5,0.5,0.5,1.0,1.0,1.0,1.0,1.0,1.0])
        
        Ct = ekf.get_jacobian_C()

        ekf.update(xHat,Qt,zt,ht,Ct)

        self.assert_states_equal(xHat,statesExpected)
        self.assert_covariances_equal(xHat,expectedP)

    def test_with_measurements(self):
        p0 = np.array([[1.0,1.0,-1.0]]).T
        q0 = np.array([[0.2,0.2,1.0,]]).T
        v0 = np.array([[1.0,2.0,0.2]]).T
        ba0 = np.zeros((3,1))
        bg0 = np.zeros((3,1))
        cov0 = np.array([[1.0,1.0,1.0]]).T
        xHat = States(p0,q0,v0,ba0,bg0,cov0,cov0,cov0,cov0,cov0)
        Qt = np.identity(7)
        zt = np.array([[1.2,1.5,-0.8,1.1,1.2,1.8,-0.1]]).T
        ht = ekf.update_measurement_model(xHat)
        # ht = np.array([[1.0,1.0,-1.0,1.0,1.0,2.0,0.2]]).T

        pExpected = np.array([[1.1000,1.2500,-0.9000]]).T
        qExpected = np.array([[0.2000,0.2000,1.0500]]).T
        vExpected = np.array([[1.1000,1.9000,0.0500]]).T
        baExpected = np.zeros((3,1))
        bgExpected = np.zeros((3,1))
        statesExpected = States(pExpected,qExpected,vExpected,baExpected,bgExpected,cov0,cov0,cov0,cov0,cov0)

        expectedP = np.diag([0.5,0.5,0.5,1.0,1.0,0.5,0.5,0.5,0.5,1.0,1.0,1.0,1.0,1.0,1.0])
        
        Ct = ekf.get_jacobian_C()

        ekf.update(xHat,Qt,zt,ht,Ct)

        self.assert_states_equal(xHat,statesExpected)
        self.assert_covariances_equal(xHat,expectedP)

    def assert_states_equal(self,xHat,statesExpected):
        for i in range(3):
            self.assertAlmostEqual(xHat.p.item(i),statesExpected.p.item(i))
            self.assertAlmostEqual(xHat.q.item(i),statesExpected.q.item(i))
            self.assertAlmostEqual(xHat.v.item(i),statesExpected.v.item(i))
            self.assertAlmostEqual(xHat.ba.item(i),statesExpected.ba.item(i))
            self.assertAlmostEqual(xHat.bg.item(i),statesExpected.bg.item(i))

    def assert_covariances_equal(self,xHat,expectedP):
        for i in range(15):
            for j in range(15):
                self.assertAlmostEqual(xHat.P[i][j],expectedP[i][j])


if __name__ == '__main__':
    unittest.main()