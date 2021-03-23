import sys
sys.path.append('../scripts')
import numpy as np

import unittest
import ekf
from states import States

class TestPropagation(unittest.TestCase):
    def test_no_inputs(self):
        p0 = np.zeros((3,1))
        th0 = np.zeros((3,1))
        v0 = np.zeros((3,1))
        ba0 = np.zeros((3,1))
        bg0 = np.zeros((3,1))
        cov0 = [1.0,1.0,1.0]
        xHat = States(p0,th0,v0,ba0,bg0,cov0,cov0,cov0,cov0,cov0)
        accel = np.zeros((3,1))
        omega = np.zeros((3,1))
        dt = 0.1
        pExpected = np.zeros((3,1))
        qExpected = np.zeros((3,1))
        vExpected = np.array([[0.0,0.0,0.981]]).T
        baExpected = np.zeros((3,1))
        bgExpected = np.zeros((3,1))
        covExpected = np.array([[1.0,1.0,1.0]]).T
        expectedStateEstimates = States(pExpected,qExpected,vExpected,baExpected,bgExpected,covExpected,covExpected,covExpected,covExpected,covExpected)
        expectedA = np.zeros((15,15))
        xHat = ekf.dynamics(xHat,accel,omega,dt)
        self.assert_state_equal(xHat,expectedStateEstimates)

    def test_inputs_equals_ones(self):
        p0 = np.zeros((3,1))
        th0 = np.zeros((3,1))
        v0 = np.zeros((3,1))
        ba0 = np.zeros((3,1))
        bg0 = np.zeros((3,1))
        cov0 = np.array([[1.0,1.0,1.0]]).T
        xHat = States(p0,th0,v0,ba0,bg0,cov0,cov0,cov0,cov0,cov0)
        accel = np.array([[1.0,1.0,1.0]]).T
        omega = np.array([[1.0,1.0,1.0]]).T
        dt = 0.1
        pExpected = np.zeros((3,1))
        thExpected = np.array([[0.1,0.1,0.1]]).T
        vExpected = np.array([[0.1,0.1,1.081]]).T
        baExpected = np.zeros((3,1))
        bgExpected = np.zeros((3,1))
        covExpected = np.array([[1.0,1.0,1.0]]).T
        expectedStateEstimates = States(pExpected,thExpected,vExpected,baExpected,bgExpected,covExpected,covExpected,covExpected,covExpected,covExpected)
        expectedA = np.zeros((15,15))
        xHat = ekf.dynamics(xHat,accel,omega,dt)
        self.assert_state_equal(xHat,expectedStateEstimates)

    def assert_state_equal(self,xHat,expectedStateEstimates):
        for i in range(3):
            self.assertAlmostEqual(xHat.p.item(i),expectedStateEstimates.p.item(i))
            self.assertAlmostEqual(xHat.th.item(i),expectedStateEstimates.th.item(i))
            self.assertAlmostEqual(xHat.v.item(i),expectedStateEstimates.v.item(i))
            self.assertAlmostEqual(xHat.ba.item(i),expectedStateEstimates.ba.item(i))
            self.assertAlmostEqual(xHat.bg.item(i),expectedStateEstimates.bg.item(i))

    def assert_jacobian_equal(self,xHat,expectedA):
        for j in range(15):
            for k in range(15):
                self.assertEqual(xHat.A[j][k],expectedA[j][k])


if __name__ == '__main__':
    unittest.main()