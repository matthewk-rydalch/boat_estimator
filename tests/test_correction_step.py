import sys
sys.path.append('/home/matt/px4_ws/src/boat_estimator/scripts')
import numpy as np

import unittest
import ekf
from states_covariance import StatesCovariance
from ekf_class import EKF

class TestUpdate(unittest.TestCase):
    def test_compass_meas_zero(self):
        cov0 = np.array([[1.0,1.0,1.0]]).T
        zt = np.zeros((1,1))
        estimator = EKF()
        ht = ekf.update_compass_measurement_model(estimator.beleif)
        C = ekf.get_jacobian_C_compass()
        estimator.run_correction_step(estimator.params.QtGpsCompass,zt,ht,C)

        pExpected = np.zeros((3,1))
        qExpected = np.zeros((3,1))
        vExpected = np.zeros((3,1))
        baExpected = np.zeros((3,1))
        bgExpected = np.zeros((3,1))
        beleifExtected = StatesCovariance(pExpected,qExpected,vExpected,baExpected,bgExpected,cov0,cov0,cov0,cov0,cov0)

        expectedP = np.diag([10.0,10.0,10.0,10.0,10.0,0.9091,10.0,10.0,10.0,10.0,10.0,10.0,10.0,10.0,10.0])

        self.assert_states_equal(estimator.beleif,statesExpected)
        self.assert_covariances_equal(estimator.beleif,expectedP)

    def test_with_gps_measurements(self):
        cov0 = np.array([[1.0,1.0,1.0]]).T
        
        zt = np.array([[0.2,0.5,-0.3,0.2,-1.0,-0.1]]).T
        estimator = EKF()
        ht = ekf.update_gps_measurement_model(estimator.beleif)
        C = ekf.get_jacobian_C_gps()
        estimator.run_correction_step(estimator.params.QtGps,zt,ht,C)

        pExpected = np.array([[0.1818,0.4545,-0.2727]]).T
        qExpected = np.array([[0.0,0.0,0.0]]).T
        vExpected = np.array([[0.1818,-0.9091,-0.0909]]).T
        baExpected = np.zeros((3,1))
        bgExpected = np.zeros((3,1))
        beleifExpected = StatesCovariance(pExpected,qExpected,vExpected,baExpected,bgExpected,cov0,cov0,cov0,cov0,cov0)

        expectedP = np.diag([0.9091,0.9091,0.9091,10.0,10.0,10.0,0.9091,0.9091,0.9091,10.0,10.0,10.0,10.0,10.0,10.0])

        self.assert_states_equal(estimator.beleif,statesExpected)
        self.assert_covariances_equal(estimator.beleif,expectedP)

    def assert_states_equal(self,beleif,statesExpected):
        decimalPlace = 3
        for i in range(3):
            self.assertAlmostEqual(beleif.p.item(i),statesExpected.p.item(i),decimalPlace)
            self.assertAlmostEqual(beleif.q.item(i),statesExpected.q.item(i),decimalPlace)
            self.assertAlmostEqual(beleif.v.item(i),statesExpected.v.item(i),decimalPlace)
            self.assertAlmostEqual(beleif.ba.item(i),statesExpected.ba.item(i),decimalPlace)
            self.assertAlmostEqual(beleif.bg.item(i),statesExpected.bg.item(i),decimalPlace)

    def assert_covariances_equal(self,beleif,expectedP):
        decimalPlace = 3
        for i in range(15):
            for j in range(15):
                self.assertAlmostEqual(beleif.P[i][j],expectedP[i][j],decimalPlace)


if __name__ == '__main__':
    unittest.main()