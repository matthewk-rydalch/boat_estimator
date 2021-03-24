import sys
sys.path.append('../scripts')
import numpy as np

import unittest
import ekf
from states import States
from sensors import GpsMsg
from ekf_class import EKF

class TestUpdate(unittest.TestCase):
    def test_with_measurements(self):
        positionEcef = [-1800198.22956286, -4532912.78883003,  4098399.82768237]
        velocityEcef = [0.0,0.0,0.0]
        gps = GpsMsg(positionEcef,velocityEcef)
        estimator = EKF()
        estimator.latRef = 40.23
        estimator.lonRef = -111.66
        estimator.altRef = 1370.0
        estimator.gps_callback(gps)

        pExpected = np.array([[0.9091,1.8182,2.7273]]).T
        qExpected = np.zeros((3,1))
        vExpected = np.zeros((3,1))
        baExpected = np.zeros((3,1))
        bgExpected = np.zeros((3,1))
        cov0 = np.array([[10.0,10.0,10.0]]).T
        statesExpected = States(pExpected,qExpected,vExpected,baExpected,bgExpected,cov0,cov0,cov0,cov0,cov0)
        expectedP = np.diag([0.9091,0.9091,0.9091,10.0,10.0,10.0,0.9091,0.9091,0.9091,10.0,10.0,10.0,10.0,10.0,10.0])
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