import sys
sys.path.append('../scripts')
import numpy as np

import unittest
import ekf
from states import States

class TestPropagation(unittest.TestCase):
    def test_propagation(self):
        p0 = [0.0,0.0,0.0]
        q0 = [0.0,0.0,0.0,1.0]
        v0 = [0.0,0.0,0.0]
        ba0 = [0.0,0.0,0.0]
        bg0 = [0.0,0.0,0.0]
        cov0 = [1.0,1.0,1.0]
        xHat = States(p0,q0,v0,ba0,bg0,cov0,cov0,cov0,cov0,cov0)
        accel = [0.0,0.0,0.0]
        omega = [0.0,0.0,0.0]
        expectedXHat = States(p0,q0,v0,ba0,bg0,cov0,cov0,cov0,cov0,cov0)
        expectedP = [[1.0,0.0,0.0],
                     [0.0,1.0,0.0],
                     [0.0,0.0,1.0]]
        [xHat, P] = ekf.propagate()
        self.assert_state_equal(xHat,expectedXHat)
        for i in range(3):
            for j in range(3):
                self.assertEqual(P[i][j],expectedP[i][j])

    def assert_state_equal(self,xHat,expectedXHat):
        for i in range(3):
            self.assertAlmostEqual(xHat.p[i],expectedXHat.p[i])
            self.assertAlmostEqual(xHat.v[i],expectedXHat.v[i])
            self.assertAlmostEqual(xHat.ba[i],expectedXHat.ba[i])
            self.assertAlmostEqual(xHat.bg[i],expectedXHat.bg[i])
        for j in range(4):
            self.assertAlmostEqual(xHat.q[j],expectedXHat.q[j])


if __name__ == '__main__':
    unittest.main()