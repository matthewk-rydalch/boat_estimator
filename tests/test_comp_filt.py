import sys
sys.path.append('/home/matt/px4_ws/src/boat_estimator/scripts/ekf')
sys.path.append('/home/matt/px4_ws/src/boat_estimator/scripts/structs')
import numpy as np

import unittest
import comp_filter
from states_covariance import StatesCovariance
from sensors import ImuMsg

class TestComplimentaryFilter(unittest.TestCase):
    def test_zero_inputs(self):
        p0 = np.zeros((3,1))
        q0 = np.zeros((3,1))
        v0 = np.zeros((3,1))
        ba0 = np.zeros((3,1))
        bg0 = np.zeros((3,1))
        cov0 = np.array([[1.0,1.0,1.0]]).T
        belief = StatesCovariance(p0,q0,v0,ba0,bg0,cov0,cov0,cov0,cov0,cov0)
        accelerometers = [0.0,0.0,9.81]
        gyros = [0.0,0.0,0.0]
        imu = ImuMsg(0.0,accelerometers,gyros)
        dt = 0.1
        kp = 0.1
        ki = 0.9
        comp_filter.run(belief,imu,dt,kp,ki)

        qExpected = np.array([[0.0,0.0,0.0]]).T

        self.assert_attitude(belief.q,qExpected)

    def test_same_inputs(self):
        p0 = np.zeros((3,1))
        q0 = np.zeros((3,1))
        v0 = np.zeros((3,1))
        ba0 = np.zeros((3,1))
        bg0 = np.zeros((3,1))
        cov0 = np.array([[1.0,1.0,1.0]]).T
        belief = StatesCovariance(p0,q0,v0,ba0,bg0,cov0,cov0,cov0,cov0,cov0)
        accelerometers = [0.0,0.0,9.81]
        gyros = [0.0,0.0,0.0]
        imu = ImuMsg(0.0,accelerometers,gyros)
        dt = 0.1
        kp = 0.1
        ki = 0.9
        comp_filter.run(belief,imu,dt,kp,ki)

        qExpected = np.array([[0.0,0.0,0.0]]).T

        self.assert_attitude(belief.q,qExpected)

    def assert_attitude(self,q,qExpected):
        decimalPlaces = 3
        for i in range(3):
            self.assertAlmostEqual(q.item(i),qExpected.item(i),decimalPlaces)


if __name__ == '__main__':
    unittest.main()