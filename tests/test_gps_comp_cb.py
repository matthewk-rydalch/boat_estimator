import sys
sys.path.append('/home/matt/px4_ws/src/boat_estimator/scripts/ekf')
sys.path.append('/home/matt/px4_ws/src/boat_estimator/scripts/structs')

import numpy as np

import unittest
import ekf
from states_covariance import StatesCovariance
from sensors import GpsCompassMsg
from estimator_class import Estimator

class TestUpdate(unittest.TestCase):
    def test_measure_zero(self):
        headingDeg = 0.0
        gpsCompass = GpsCompassMsg(headingDeg)
        estimator = Estimator()
        estimator.gps_compass_callback(gpsCompass)

        pExpected = np.zeros((3,1))
        qExpected = np.array([[0.0,0.0,0.0]]).T
        vExpected = np.zeros((3,1))
        baExpected = np.zeros((3,1))
        bgExpected = np.zeros((3,1))
        cov0 = np.array([[10.0,10.0,10.0]]).T
        beliefExpected = StatesCovariance(pExpected,qExpected,vExpected,baExpected,bgExpected,cov0,cov0,cov0,cov0,cov0)
        expectedP = np.diag([10.0,10.0,10.0,10.0,10.0,0.9091,10.0,10.0,10.0,10.0,10.0,10.0,10.0,10.0,10.0])
        self.assert_states_equal(estimator.belief,beliefExpected)
        self.assert_covariances_equal(estimator.belief,expectedP)

    def test_measure_p8(self):
        headingDeg = 0.8
        gpsCompass = GpsCompassMsg(headingDeg)
        estimator = Estimator()
        estimator.gps_compass_callback(gpsCompass)

        pExpected = np.zeros((3,1))
        qExpected = np.array([[0.0,0.0,0.7273]]).T
        vExpected = np.zeros((3,1))
        baExpected = np.zeros((3,1))
        bgExpected = np.zeros((3,1))
        cov0 = np.array([[10.0,10.0,10.0]]).T
        beliefExpected = StatesCovariance(pExpected,qExpected,vExpected,baExpected,bgExpected,cov0,cov0,cov0,cov0,cov0)
        expectedP = np.diag([10.0,10.0,10.0,10.0,10.0,0.9091,10.0,10.0,10.0,10.0,10.0,10.0,10.0,10.0,10.0])
        self.assert_states_equal(estimator.belief,beliefExpected)
        self.assert_covariances_equal(estimator.belief,expectedP)

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