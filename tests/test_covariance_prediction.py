import sys
sys.path.append('/home/matt/px4_ws/src/boat_estimator/scripts')
import numpy as np

import unittest
import ekf
from states import States
from ekf_class import EKF

class TestPredictionStep(unittest.TestCase):
    def test_no_tests(sefl):
        print('not a good way to test this right now!!!!!!!!!!!!')
    #Right now I do not know a good way to test this.
    
    # def test_no_movement(self):
    #     accelerometers = np.array([[0.0,0.0,-9.81]]).T
    #     gyros = np.array([[0.0,0.0,0.0]]).T
    #     u = [accelerometers,gyros]
    #     dt = 0.1
    #     estimator = EKF()
    #     steps = 5
    #     for i in range(steps):
    #         PPrev = estimator.xHat.P
    #         estimator.prediction_step(u,dt)
    #         self.check_covariance_increases(estimator.xHat,PPrev)

    # def test_chaotic_movement(self):
    #     accelerometers = np.array([[1.2,22.1,-17.3]]).T
    #     gyros = np.array([[4.0,-3.1,1.0]]).T
    #     u = [accelerometers,gyros]
    #     dt = 0.1
    #     estimator = EKF()
    #     steps = 5
    #     for i in range(steps):
    #         PPrev = estimator.xHat.P
    #         estimator.prediction_step(u,dt)
    #         self.check_covariance_increases(estimator.xHat,PPrev)

    # def check_covariance_increases(self,xHat,PPrev):
    #     for i in range(15):
    #         for j in range(15):
    #             self.assertGreaterEqual(xHat.P[i][j],PPrev[i][j])

if __name__ == '__main__':
    unittest.main()