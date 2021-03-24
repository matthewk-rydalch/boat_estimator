import sys
sys.path.append('../scripts')
import numpy as np

import unittest
import ekf
from states import States
from sensors import GpsMsg
from ekf_class import EKF

class TestUpdate(unittest.TestCase):
    def test_all_zeros(self):
        positionEcef = [0.0,0.0,0.0]
        velocityEcef = [0.0,0.0,0.0]
        gps = GpsMsg(positionEcef,velocityEcef)
        estimator = EKF()
        estimator.latRef = 0.0
        estimator.lonRef = 0.0
        estimator.altRef = 0.0
        estimator.gps_callback(gps)

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