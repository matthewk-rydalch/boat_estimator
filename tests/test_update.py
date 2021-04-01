import sys
sys.path.append('/home/matt/px4_ws/src/boat_estimator/scripts/estimator')
sys.path.append('/home/matt/px4_ws/src/boat_estimator/scripts/structs')
import numpy as np

import unittest
import ekf
from states_covariance import StatesCovariance

class TestUpdate(unittest.TestCase):
    def test_compass_meas_zero(self):
        p0 = np.zeros((3,1))
        q0 = np.zeros((3,1))
        v0 = np.zeros((3,1))
        ba0 = np.zeros((3,1))
        bg0 = np.zeros((3,1))
        cov0 = np.array([[10.0,10.0,10.0]]).T
        belief = StatesCovariance(p0,q0,v0,ba0,bg0,cov0,cov0,cov0,cov0,cov0)
        Qt = np.identity(1)
        zt = np.zeros((1,1))
        ht = ekf.update_compass_measurement_model(belief)

        pExpected = np.zeros((3,1))
        qExpected = np.zeros((3,1))
        vExpected = np.zeros((3,1))
        baExpected = np.zeros((3,1))
        bgExpected = np.zeros((3,1))
        beliefExpected = StatesCovariance(pExpected,qExpected,vExpected,baExpected,bgExpected,cov0,cov0,cov0,cov0,cov0)

        expectedP = np.diag([10.0,10.0,10.0,10.0,10.0,0.9091,10.0,10.0,10.0,10.0,10.0,10.0,10.0,10.0,10.0])
        
        Ct = ekf.get_jacobian_C_compass()

        ekf.update(belief,Qt,zt,ht,Ct)

        self.assert_states_equal(belief,beliefExpected)
        self.assert_covariances_equal(belief,expectedP)

    def test_with_gps_measurements(self):
        p0 = np.array([[1.0,1.0,-1.0]]).T
        q0 = np.array([[0.2,0.2,1.0]]).T
        v0 = np.array([[1.0,2.0,0.2]]).T
        ba0 = np.zeros((3,1))
        bg0 = np.zeros((3,1))
        cov0 = np.array([[10.0,10.0,10.0]]).T
        belief = StatesCovariance(p0,q0,v0,ba0,bg0,cov0,cov0,cov0,cov0,cov0)
        Qt = np.identity(6)
        zt = np.array([[1.2,1.5,-0.8,1.2,1.8,-0.1]]).T
        ht = ekf.update_gps_measurement_model(belief)

        pExpected = np.array([[1.1818,1.4545,-0.8182]]).T
        qExpected = np.array([[0.2000,0.2000,1.0]]).T
        vExpected = np.array([[1.1818,1.8182,-0.0727]]).T
        baExpected = np.zeros((3,1))
        bgExpected = np.zeros((3,1))
        beliefExpected = StatesCovariance(pExpected,qExpected,vExpected,baExpected,bgExpected,cov0,cov0,cov0,cov0,cov0)

        expectedP = np.diag([0.9091,0.9091,0.9091,10.0,10.0,10.0,0.9091,0.9091,0.9091,10.0,10.0,10.0,10.0,10.0,10.0])
        
        Ct = ekf.get_jacobian_C_gps()

        ekf.update(belief,Qt,zt,ht,Ct)

        self.assert_states_equal(belief,beliefExpected)
        self.assert_covariances_equal(belief,expectedP)

    def assert_states_equal(self,belief,beliefExpected):
        decimalPlaces = 3
        for i in range(3):
            self.assertAlmostEqual(belief.p.item(i),beliefExpected.p.item(i),decimalPlaces)
            self.assertAlmostEqual(belief.q.item(i),beliefExpected.q.item(i),decimalPlaces)
            self.assertAlmostEqual(belief.v.item(i),beliefExpected.v.item(i),decimalPlaces)
            self.assertAlmostEqual(belief.ba.item(i),beliefExpected.ba.item(i),decimalPlaces)
            self.assertAlmostEqual(belief.bg.item(i),beliefExpected.bg.item(i),decimalPlaces)

    def assert_covariances_equal(self,belief,expectedP):
        decimalPlaces = 3
        for i in range(15):
            for j in range(15):
                self.assertAlmostEqual(belief.P[i][j],expectedP[i][j],decimalPlaces)


if __name__ == '__main__':
    unittest.main()