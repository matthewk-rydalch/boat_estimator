import sys
sys.path.append('/home/matt/px4_ws/src/boat_estimator/scripts/estimator')
sys.path.append('/home/matt/px4_ws/src/boat_estimator/scripts/structs')
import numpy as np
from scipy.spatial.transform import Rotation as R

import unittest
import ekf
from states_covariance import StatesCovariance

class TestJacobianA(unittest.TestCase):
    def test_no_inputs(self):
        p = np.zeros((3,1))
        q = np.zeros((3,1))
        v = np.array([[0.0,0.0,0.981]]).T
        ba = np.zeros((3,1))
        bg = np.zeros((3,1))
        cov = np.identity(15)
        belief = StatesCovariance(p,q,v,ba,bg,cov)
        accelerometers = np.zeros((3,1))
        gyros = np.zeros((3,1))
        u = [accelerometers,gyros]
        expectedA = np.zeros((15,15))
        expectedA[0][4] = 0.981
        expectedA[0:3,6:9] = np.identity(3)
        expectedA[3:6,12:15] = -np.identity(3)
        expectedA[6][4] = 9.81
        expectedA[7][3] = -9.81
        expectedA[6:9,9:12] = -np.identity(3)
        Bt = ekf.update_jacobian_B(belief)
        # self.assert_jacobian_equal(At,expectedA)

    def assert_jacobian_equal(self,At,expectedA):
        for j in range(15):
            for k in range(15):
                self.assertAlmostEqual(At[j][k],expectedA[j][k])

if __name__ == '__main__':
    unittest.main()