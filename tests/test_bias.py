import sys
sys.path.append('/home/matt/px4_ws/src/boat_estimator/scripts/ekf')
sys.path.append('/home/matt/px4_ws/src/boat_estimator/scripts/structs')
import numpy as np

import unittest
import ekf
from states_covariance import StatesCovariance
from ekf_class import EKF
from sensors import ImuMsg
from sensors import GpsMsg

class TestBias(unittest.TestCase):
    def test_pitch_xz_acceleration(self):
        time = 123.023
        accelerometers = [0.0,0.0,-9.81]
        gyros = [0.0,1.0,0.0]
        imu = ImuMsg(time,accelerometers,gyros)
        dt = 0.1

        estimator = EKF()
        estimator.imuPrevTime = time
        steps = 5
        for i in range(steps):
            imu.time = estimator.imuPrevTime + dt
            pPrev = estimator.beleif.p
            qPrev = estimator.beleif.q
            vPrev = estimator.beleif.v
            baPrev = estimator.beleif.ba
            bgPrev = estimator.beleif.bg
            estimator.imu_callback(imu)
        positionEcef = [-1800198.22956286, -4532912.78883003,  4098399.82768237]
        velocityEcef = [0.0,0.0,0.0]
        gps = GpsMsg(positionEcef,velocityEcef)
        estimator.set_ref_lla_callback(40.23,-111.66,1370.0)
        estimator.gps_callback(gps)
        print('accelerometer bias = ', estimator.beleif.ba)
        print('gyro bias = ', estimator.beleif.bg)

if __name__ == '__main__':
    unittest.main()