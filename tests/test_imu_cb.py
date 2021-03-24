import sys
sys.path.append('/home/matt/px4_ws/src/boat_estimator/scripts')
import numpy as np

import unittest
import ekf
from states import States
from ekf_class import EKF
from sensors import ImuMsg

class TestPredictionStep(unittest.TestCase):
    def test_imu_cb(self):
        timeS = 123.123
        accelerometers = [0.1,0.5,-0.2]
        gyros = [1.1,-0.3,0.1]
        imu = ImuMsg(timeS,accelerometers,gyros)

        estimator = EKF()
        estimator.imuPrevTime = 123.023
        estimator.imu_callback(imu)

        print('state = ', estimator.xHat.p, ' ', estimator.xHat.q, ' ', estimator.xHat.v)
        print('covariance = ', estimator.xHat.P)
        #TODO: May want to actually make this a complete test later.  For now I just wanted to make sure things were running.

if __name__ == '__main__':
    unittest.main()