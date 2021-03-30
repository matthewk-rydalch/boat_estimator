import sys
sys.path.append('/home/matt/px4_ws/src/boat_estimator/scripts/ekf')
sys.path.append('/home/matt/px4_ws/src/boat_estimator/scripts/structs')
import numpy as np

import unittest
import comp_filter
from states_covariance import StatesCovariance
from sensors import ImuMsg

class TestComplimentaryFilter(unittest.TestCase):
    def test_comp_filter(self):
        p0 = np.zeros((3,1))
        q0 = np.zeros((3,1))
        v0 = np.zeros((3,1))
        ba0 = np.zeros((3,1))
        bg0 = np.zeros((3,1))
        cov0 = np.array([[1.0,1.0,1.0]]).T
        beleif = StatesCovariance(p0,q0,v0,ba0,bg0,cov0,cov0,cov0,cov0,cov0)
        accelerometers = [0.0,0.0,9.81]
        gyros = [0.0,0.0,0.0]
        imu = ImuMsg(0.0,accelerometers,gyros)
        dt = 0.1
        alpha = 1.0
        comp_filter.run(beleif,imu,dt,alpha)

        qExpected = np.array([[1.0,1.0,0.0]]).T


if __name__ == '__main__':
    unittest.main()