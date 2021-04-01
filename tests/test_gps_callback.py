import sys
sys.path.append('/home/matt/px4_ws/src/boat_estimator/scripts/estimator')
sys.path.append('/home/matt/px4_ws/src/boat_estimator/scripts/structs')
import numpy as np

import unittest
import ekf
from states_covariance import StatesCovariance
from sensors import GpsMsg
from estimator_class import Estimator

class TestUpdate(unittest.TestCase):
    def test_ref_lla_set(self):
        positionEcef = [-1800198.22956286, -4532912.78883003,  4098399.82768237]
        velocityEcef = [0.0,0.0,0.0]
        gps = GpsMsg(positionEcef,velocityEcef)
        estimator = Estimator()
        estimator.latRef = 40.23
        estimator.lonRef = -111.66
        estimator.altRef = 1370.0
        estimator.gps_callback(gps)

        pExpected = np.zeros((3,1))
        qExpected = np.zeros((3,1))
        vExpected = np.zeros((3,1))
        baExpected = np.zeros((3,1))
        bgExpected = np.zeros((3,1))
        cov0 = np.array([[10.0,10.0,10.0]]).T
        beliefExpected = StatesCovariance(pExpected,qExpected,vExpected,baExpected,bgExpected,cov0,cov0,cov0,cov0,cov0)
        expectedP = np.diag([10.0,10.0,10.0,10.0,10.0,10.0,10.0,10.0,10.0,10.0,10.0,10.0,10.0,10.0,10.0])
        self.assert_states_equal(estimator.belief,beliefExpected)
        self.assert_covariances_equal(estimator.belief,expectedP)

    def test_with_measurements(self):
        positionEcef = [-1800198.22956286, -4532912.78883003,  4098399.82768237]
        velocityEcef = [0.0,0.0,0.0]
        gps = GpsMsg(positionEcef,velocityEcef)
        estimator = Estimator()
        estimator.set_ref_lla_callback(40.23,-111.66,1370.0)
        estimator.gps_callback(gps)

        pExpected = np.array([[0.9091,1.8182,2.7273]]).T
        qExpected = np.zeros((3,1))
        vExpected = np.zeros((3,1))
        baExpected = np.zeros((3,1))
        bgExpected = np.zeros((3,1))
        cov0 = np.array([[10.0,10.0,10.0]]).T
        beliefExpected = StatesCovariance(pExpected,qExpected,vExpected,baExpected,bgExpected,cov0,cov0,cov0,cov0,cov0)
        expectedP = np.diag([0.9091,0.9091,0.9091,10.0,10.0,10.0,0.9091,0.9091,0.9091,10.0,10.0,10.0,10.0,10.0,10.0])
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