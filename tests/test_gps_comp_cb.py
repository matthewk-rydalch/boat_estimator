import sys
sys.path.append('../scripts')
import numpy as np

import unittest
import ekf
from states import States
from sensors import GpsCompassMsg
from ekf_class import EKF

class TestUpdate(unittest.TestCase):
    def test_measure_zero(self):
        headingDeg = 0.0
        gpsCompass = GpsCompassMsg(headingDeg)
        estimator = EKF()
        estimator.gps_compass_callback(gpsCompass)

        pExpected = np.zeros((3,1))
        qExpected = np.array([[0.0,0.0,0.0]]).T
        vExpected = np.zeros((3,1))
        baExpected = np.zeros((3,1))
        bgExpected = np.zeros((3,1))
        cov0 = np.array([[10.0,10.0,10.0]]).T
        statesExpected = States(pExpected,qExpected,vExpected,baExpected,bgExpected,cov0,cov0,cov0,cov0,cov0)
        expectedP = np.diag([10.0,10.0,10.0,10.0,10.0,0.9091,10.0,10.0,10.0,10.0,10.0,10.0,10.0,10.0,10.0])
        self.assert_states_equal(estimator.xHat,statesExpected)
        self.assert_covariances_equal(estimator.xHat,expectedP)

    def test_measure_p8(self):
        headingDeg = 0.8
        gpsCompass = GpsCompassMsg(headingDeg)
        estimator = EKF()
        estimator.gps_compass_callback(gpsCompass)

        pExpected = np.zeros((3,1))
        qExpected = np.array([[0.0,0.0,0.7273]]).T
        vExpected = np.zeros((3,1))
        baExpected = np.zeros((3,1))
        bgExpected = np.zeros((3,1))
        cov0 = np.array([[10.0,10.0,10.0]]).T
        statesExpected = States(pExpected,qExpected,vExpected,baExpected,bgExpected,cov0,cov0,cov0,cov0,cov0)
        expectedP = np.diag([10.0,10.0,10.0,10.0,10.0,0.9091,10.0,10.0,10.0,10.0,10.0,10.0,10.0,10.0,10.0])
        self.assert_states_equal(estimator.xHat,statesExpected)
        self.assert_covariances_equal(estimator.xHat,expectedP)

    def assert_states_equal(self,xHat,statesExpected):
        decimalPlaces = 3
        for i in range(3):
            self.assertAlmostEqual(xHat.p.item(i),statesExpected.p.item(i),decimalPlaces)
            self.assertAlmostEqual(xHat.q.item(i),statesExpected.q.item(i),decimalPlaces)
            self.assertAlmostEqual(xHat.v.item(i),statesExpected.v.item(i),decimalPlaces)
            self.assertAlmostEqual(xHat.ba.item(i),statesExpected.ba.item(i),decimalPlaces)
            self.assertAlmostEqual(xHat.bg.item(i),statesExpected.bg.item(i),decimalPlaces)

    def assert_covariances_equal(self,xHat,expectedP):
        decimalPlaces = 3
        for i in range(15):
            for j in range(15):
                self.assertAlmostEqual(xHat.P[i][j],expectedP[i][j],decimalPlaces)


if __name__ == '__main__':
    unittest.main()