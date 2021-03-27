import sys
sys.path.append('/home/matt/px4_ws/src/boat_estimator/scripts/misc')
sys.path.append('/home/matt/px4_ws/src/boat_estimator/scripts/structs')
import numpy as np

import unittest
import synthetic_measurements
from sensors import TruthMsg
from sensors import ImuMsg

class TestDynamics(unittest.TestCase):
    def test_random(self):
        t = 0.0 #not important
        position = [-3.0,1.0,-1.0]
        orientation = [0.2,0.3,-0.1]
        velocity = [-2.96377917,  1.48202406,  0.14006332]
        angularVelocity = [0.0,0.0,0.0]
        acceleration = [0.0,0.0,0.0]
        truth = TruthMsg(t,position,orientation,velocity,angularVelocity,acceleration)
        imu = ImuMsg()
        accelerometers = [2.89905323, -1.86189936, -9.1850379]
        gyros = [0.0,0.0,0.0]
        expectedImu = ImuMsg(t,accelerometers,gyros)
        synthetic_measurements.compute_imu(truth,imu)
        self.assert_imus(imu,expectedImu)

    def test_zeros(self):
        t = 0.0 #not important
        position = [0.0,0.0,0.0]
        orientation = [0.0,0.0,0.0]
        velocity = [0.0,0.0,0.0]
        angularVelocity = [0.0,0.0,0.0]
        acceleration = [0.0,0.0,0.0]
        truth = TruthMsg(t,position,orientation,velocity,angularVelocity,acceleration)
        imu = ImuMsg()
        accelerometers = [0.0,0.0,-9.81]
        gyros = [0.0,0.0,0.0]
        expectedImu = ImuMsg(t,accelerometers,gyros)
        synthetic_measurements.compute_imu(truth,imu)
        self.assert_imus(imu,expectedImu)

    def test_orientation_only(self):
        t = 0.0 #not important
        position = [0.0,0.0,0.0]
        orientation = [1.2,-0.3,2.2]
        velocity = [0.0,0.0,0.0]
        angularVelocity = [0.0,0.0,0.0]
        acceleration = [0.0,0.0,0.0]
        truth = TruthMsg(t,position,orientation,velocity,angularVelocity,acceleration)
        imu = ImuMsg()
        accelerometers = [-2.8991,-8.7349,-3.3960]
        gyros = [0.0,0.0,0.0]
        expectedImu = ImuMsg(t,accelerometers,gyros)
        synthetic_measurements.compute_imu(truth,imu)
        self.assert_imus(imu,expectedImu)

    def test_orientation_and_angular_velocity(self):
        t = 0.0 #not important
        position = [0.0,0.0,0.0]
        orientation = [1.2,-0.3,2.2]
        velocity = [0.0,0.0,0.0]
        angularVelocity = [2.3,-0.1,0.44]
        acceleration = [0.0,0.0,0.0]
        truth = TruthMsg(t,position,orientation,velocity,acceleration,angularVelocity)
        imu = ImuMsg()
        accelerometers = [-2.8991,-8.7349,-3.3960]
        gyros = [2.3,-0.1,0.44]
        expectedImu = ImuMsg(t,accelerometers,gyros)
        synthetic_measurements.compute_imu(truth,imu)
        self.assert_imus(imu,expectedImu)

    def test_orientation_angular_position_velocity(self):
        t = 0.0 #not important
        position = [1.3,43.2,-6.5]
        orientation = [1.2,-0.3,2.2]
        velocity = [3.3,-0.4,-2.1]
        angularVelocity = [2.3,-0.1,0.44]
        acceleration = [0.0,0.0,0.0]
        truth = TruthMsg(t,position,orientation,velocity,acceleration,angularVelocity)
        imu = ImuMsg()
        accelerometers = [-2.8991,-8.7349,-3.3960]
        gyros = [2.3,-0.1,0.44]
        expectedImu = ImuMsg(t,accelerometers,gyros)
        synthetic_measurements.compute_imu(truth,imu)
        self.assert_imus(imu,expectedImu)

    def assert_imus(self,imu,expectedImu):
        for i in range(3):
            decimalPlaces = 3
            self.assertAlmostEqual(imu.accelerometers.item(i),expectedImu.accelerometers.item(i),decimalPlaces)
            self.assertAlmostEqual(imu.gyros.item(i),expectedImu.gyros.item(i),decimalPlaces)
 
if __name__ == '__main__':
    unittest.main()