import sys
sys.path.append('/home/matt/px4_ws/src/boat_estimator/scripts/estimator')
sys.path.append('/home/matt/px4_ws/src/boat_estimator/scripts/structs')
import numpy as np

import unittest
from states_covariance import StatesCovariance
from estimator_class import Estimator
from sensors import ImuMsg

class TestPredictionStep(unittest.TestCase):
    def test_acceleration(self):
        time = 123.023
        accelerometers = [1.0,-1.0,-9.0]
        gyros = [0.0,0.0,0.0]
        imu = ImuMsg(time,accelerometers,gyros)
        dt = 0.1

        estimator = Estimator()
        estimator.imuPrevTime = time
        steps = 5
        for i in range(steps):
            imu.time = estimator.imuPrevTime + dt
            pPrev = estimator.belief.p
            qPrev = estimator.belief.q
            vPrev = estimator.belief.v
            baPrev = estimator.belief.ba
            bgPrev = estimator.belief.bg
            estimator.imu_callback(imu)
            self.check_acceleration_trend(estimator.belief,pPrev,qPrev,vPrev,baPrev,bgPrev)

    def test_yaw(self):
        time = 123.023
        accelerometers = [0.0,0.0,-9.81]
        gyros = [0.0,0.0,1.0]
        imu = ImuMsg(time,accelerometers,gyros)
        dt = 0.1

        estimator = Estimator()
        estimator.imuPrevTime = time
        steps = 5
        for i in range(steps):
            imu.time = estimator.imuPrevTime + dt
            pPrev = estimator.belief.p
            qPrev = estimator.belief.q
            vPrev = estimator.belief.v
            baPrev = estimator.belief.ba
            bgPrev = estimator.belief.bg
            estimator.imu_callback(imu)
            self.check_yaw_trend(estimator.belief,pPrev,qPrev,vPrev,baPrev,bgPrev)

    def test_pitch_xz_acceleration(self):
        time = 123.023
        accelerometers = [0.0,0.0,-9.81]
        gyros = [0.0,1.0,0.0]
        imu = ImuMsg(time,accelerometers,gyros)
        dt = 0.1

        estimator = Estimator()
        estimator.imuPrevTime = time
        steps = 5
        for i in range(steps):
            imu.time = estimator.imuPrevTime + dt
            pPrev = estimator.belief.p
            qPrev = estimator.belief.q
            vPrev = estimator.belief.v
            baPrev = estimator.belief.ba
            bgPrev = estimator.belief.bg
            estimator.imu_callback(imu)
            self.check_pitch_trend(estimator.belief,pPrev,qPrev,vPrev,baPrev,bgPrev)

    def test_roll_yz_acceleration(self):
        time = 123.023
        accelerometers = [0.0,0.0,-9.81]
        gyros = [1.0,0.0,0.0]
        imu = ImuMsg(time,accelerometers,gyros)
        dt = 0.1

        estimator = Estimator()
        estimator.imuPrevTime = time
        steps = 5
        for i in range(steps):
            imu.time = estimator.imuPrevTime + dt
            pPrev = estimator.belief.p
            qPrev = estimator.belief.q
            vPrev = estimator.belief.v
            baPrev = estimator.belief.ba
            bgPrev = estimator.belief.bg
            estimator.imu_callback(imu)
            self.check_roll_trend(estimator.belief,pPrev,qPrev,vPrev,baPrev,bgPrev)

    def check_acceleration_trend(self,belief,pPrev,qPrev,vPrev,baPrev,bgPrev):
        self.assertGreaterEqual(belief.p.item(0),pPrev.item(0))
        self.assertGreaterEqual(belief.p.item(2),pPrev.item(2))

        self.assertGreater(belief.v.item(0),vPrev.item(0))
        self.assertGreater(belief.v.item(2),vPrev.item(2))

        self.assertLessEqual(belief.p.item(1),pPrev.item(1))

        self.assertLess(belief.v.item(1),vPrev.item(1))

        self.assertAlmostEqual(belief.q.item(0),qPrev.item(0))
        self.assertAlmostEqual(belief.q.item(1),qPrev.item(1))
        self.assertAlmostEqual(belief.q.item(2),qPrev.item(2))
        self.assertAlmostEqual(belief.ba.item(0),baPrev.item(0))
        self.assertAlmostEqual(belief.ba.item(1),baPrev.item(1))
        self.assertAlmostEqual(belief.ba.item(2),baPrev.item(2))
        self.assertAlmostEqual(belief.bg.item(0),bgPrev.item(0))
        self.assertAlmostEqual(belief.bg.item(1),bgPrev.item(1))
        self.assertAlmostEqual(belief.bg.item(2),bgPrev.item(2))

    def check_yaw_trend(self,belief,pPrev,qPrev,vPrev,baPrev,bgPrev):
        self.assertGreater(belief.q.item(2),qPrev.item(2))

        self.assertAlmostEqual(belief.p.item(0),pPrev.item(0))
        self.assertAlmostEqual(belief.p.item(1),pPrev.item(1))
        self.assertAlmostEqual(belief.p.item(2),pPrev.item(2))
        self.assertAlmostEqual(belief.q.item(0),qPrev.item(0))
        self.assertAlmostEqual(belief.q.item(1),qPrev.item(1))
        self.assertAlmostEqual(belief.v.item(0),vPrev.item(0))
        self.assertAlmostEqual(belief.v.item(1),vPrev.item(1))
        self.assertAlmostEqual(belief.v.item(2),vPrev.item(2))
        self.assertAlmostEqual(belief.ba.item(0),baPrev.item(0))
        self.assertAlmostEqual(belief.ba.item(1),baPrev.item(1))
        self.assertAlmostEqual(belief.ba.item(2),baPrev.item(2))
        self.assertAlmostEqual(belief.bg.item(0),bgPrev.item(0))
        self.assertAlmostEqual(belief.bg.item(1),bgPrev.item(1))
        self.assertAlmostEqual(belief.bg.item(2),bgPrev.item(2))


    def check_pitch_trend(self,belief,pPrev,qPrev,vPrev,baPrev,bgPrev):
        self.assertGreaterEqual(belief.p.item(2),pPrev.item(2))

        self.assertGreater(belief.q.item(1),qPrev.item(1))

        self.assertLessEqual(belief.p.item(0),pPrev.item(0))
        self.assertLessEqual(belief.v.item(0),vPrev.item(0))
        self.assertLessEqual(belief.v.item(2),vPrev.item(2))

        self.assertAlmostEqual(belief.p.item(1),pPrev.item(1))
        self.assertAlmostEqual(belief.q.item(0),qPrev.item(0))
        self.assertAlmostEqual(belief.q.item(2),qPrev.item(2))
        self.assertAlmostEqual(belief.v.item(1),vPrev.item(1))
        self.assertAlmostEqual(belief.ba.item(0),baPrev.item(0))
        self.assertAlmostEqual(belief.ba.item(1),baPrev.item(1))
        self.assertAlmostEqual(belief.ba.item(2),baPrev.item(2))
        self.assertAlmostEqual(belief.bg.item(0),bgPrev.item(0))
        self.assertAlmostEqual(belief.bg.item(1),bgPrev.item(1))
        self.assertAlmostEqual(belief.bg.item(2),bgPrev.item(2))

    def check_roll_trend(self,belief,pPrev,qPrev,vPrev,baPrev,bgPrev):
        self.assertGreaterEqual(belief.p.item(1),pPrev.item(1))
        self.assertGreaterEqual(belief.p.item(2),pPrev.item(2))
        self.assertGreaterEqual(belief.v.item(1),vPrev.item(1))

        self.assertGreater(belief.q.item(0),qPrev.item(0))

        self.assertLessEqual(belief.v.item(2),vPrev.item(2))

        self.assertAlmostEqual(belief.p.item(0),pPrev.item(0))
        self.assertAlmostEqual(belief.q.item(1),qPrev.item(1))
        self.assertAlmostEqual(belief.q.item(2),qPrev.item(2))
        self.assertAlmostEqual(belief.v.item(0),vPrev.item(0))
        self.assertAlmostEqual(belief.ba.item(0),baPrev.item(0))
        self.assertAlmostEqual(belief.ba.item(1),baPrev.item(1))
        self.assertAlmostEqual(belief.ba.item(2),baPrev.item(2))
        self.assertAlmostEqual(belief.bg.item(0),bgPrev.item(0))
        self.assertAlmostEqual(belief.bg.item(1),bgPrev.item(1))
        self.assertAlmostEqual(belief.bg.item(2),bgPrev.item(2))

if __name__ == '__main__':
    unittest.main()