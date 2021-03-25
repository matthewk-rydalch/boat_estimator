import sys
sys.path.append('/home/matt/px4_ws/src/boat_estimator/scripts/ekf')
sys.path.append('/home/matt/px4_ws/src/boat_estimator/scripts/structs')
import numpy as np

import unittest
import ekf
from states_covariance import StatesCovariance
from ekf_class import EKF
from sensors import ImuMsg

class TestPredictionStep(unittest.TestCase):
    def test_acceleration(self):
        time = 123.023
        accelerometers = [1.0,-1.0,-9.0]
        gyros = [0.0,0.0,0.0]
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
            self.check_acceleration_trend(estimator.beleif,pPrev,qPrev,vPrev,baPrev,bgPrev)

    def test_yaw(self):
        time = 123.023
        accelerometers = [0.0,0.0,-9.81]
        gyros = [0.0,0.0,1.0]
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
            self.check_yaw_trend(estimator.beleif,pPrev,qPrev,vPrev,baPrev,bgPrev)

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
            self.check_pitch_trend(estimator.beleif,pPrev,qPrev,vPrev,baPrev,bgPrev)

    def test_roll_yz_acceleration(self):
        time = 123.023
        accelerometers = [0.0,0.0,-9.81]
        gyros = [1.0,0.0,0.0]
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
            self.check_roll_trend(estimator.beleif,pPrev,qPrev,vPrev,baPrev,bgPrev)

    def check_acceleration_trend(self,beleif,pPrev,qPrev,vPrev,baPrev,bgPrev):
        self.assertGreaterEqual(beleif.p.item(0),pPrev.item(0))
        self.assertGreaterEqual(beleif.p.item(2),pPrev.item(2))

        self.assertGreater(beleif.v.item(0),vPrev.item(0))
        self.assertGreater(beleif.v.item(2),vPrev.item(2))

        self.assertLessEqual(beleif.p.item(1),pPrev.item(1))

        self.assertLess(beleif.v.item(1),vPrev.item(1))

        self.assertAlmostEqual(beleif.q.item(0),qPrev.item(0))
        self.assertAlmostEqual(beleif.q.item(1),qPrev.item(1))
        self.assertAlmostEqual(beleif.q.item(2),qPrev.item(2))
        self.assertAlmostEqual(beleif.ba.item(0),baPrev.item(0))
        self.assertAlmostEqual(beleif.ba.item(1),baPrev.item(1))
        self.assertAlmostEqual(beleif.ba.item(2),baPrev.item(2))
        self.assertAlmostEqual(beleif.bg.item(0),bgPrev.item(0))
        self.assertAlmostEqual(beleif.bg.item(1),bgPrev.item(1))
        self.assertAlmostEqual(beleif.bg.item(2),bgPrev.item(2))

    def check_yaw_trend(self,beleif,pPrev,qPrev,vPrev,baPrev,bgPrev):
        self.assertGreater(beleif.q.item(2),qPrev.item(2))

        self.assertAlmostEqual(beleif.p.item(0),pPrev.item(0))
        self.assertAlmostEqual(beleif.p.item(1),pPrev.item(1))
        self.assertAlmostEqual(beleif.p.item(2),pPrev.item(2))
        self.assertAlmostEqual(beleif.q.item(0),qPrev.item(0))
        self.assertAlmostEqual(beleif.q.item(1),qPrev.item(1))
        self.assertAlmostEqual(beleif.v.item(0),vPrev.item(0))
        self.assertAlmostEqual(beleif.v.item(1),vPrev.item(1))
        self.assertAlmostEqual(beleif.v.item(2),vPrev.item(2))
        self.assertAlmostEqual(beleif.ba.item(0),baPrev.item(0))
        self.assertAlmostEqual(beleif.ba.item(1),baPrev.item(1))
        self.assertAlmostEqual(beleif.ba.item(2),baPrev.item(2))
        self.assertAlmostEqual(beleif.bg.item(0),bgPrev.item(0))
        self.assertAlmostEqual(beleif.bg.item(1),bgPrev.item(1))
        self.assertAlmostEqual(beleif.bg.item(2),bgPrev.item(2))


    def check_pitch_trend(self,beleif,pPrev,qPrev,vPrev,baPrev,bgPrev):
        self.assertGreaterEqual(beleif.p.item(2),pPrev.item(2))

        self.assertGreater(beleif.q.item(1),qPrev.item(1))

        self.assertLessEqual(beleif.p.item(0),pPrev.item(0))
        self.assertLessEqual(beleif.v.item(0),vPrev.item(0))
        self.assertLessEqual(beleif.v.item(2),vPrev.item(2))

        self.assertAlmostEqual(beleif.p.item(1),pPrev.item(1))
        self.assertAlmostEqual(beleif.q.item(0),qPrev.item(0))
        self.assertAlmostEqual(beleif.q.item(2),qPrev.item(2))
        self.assertAlmostEqual(beleif.v.item(1),vPrev.item(1))
        self.assertAlmostEqual(beleif.ba.item(0),baPrev.item(0))
        self.assertAlmostEqual(beleif.ba.item(1),baPrev.item(1))
        self.assertAlmostEqual(beleif.ba.item(2),baPrev.item(2))
        self.assertAlmostEqual(beleif.bg.item(0),bgPrev.item(0))
        self.assertAlmostEqual(beleif.bg.item(1),bgPrev.item(1))
        self.assertAlmostEqual(beleif.bg.item(2),bgPrev.item(2))

    def check_roll_trend(self,beleif,pPrev,qPrev,vPrev,baPrev,bgPrev):
        self.assertGreaterEqual(beleif.p.item(1),pPrev.item(1))
        self.assertGreaterEqual(beleif.p.item(2),pPrev.item(2))
        self.assertGreaterEqual(beleif.v.item(1),vPrev.item(1))

        self.assertGreater(beleif.q.item(0),qPrev.item(0))

        self.assertLessEqual(beleif.v.item(2),vPrev.item(2))

        self.assertAlmostEqual(beleif.p.item(0),pPrev.item(0))
        self.assertAlmostEqual(beleif.q.item(1),qPrev.item(1))
        self.assertAlmostEqual(beleif.q.item(2),qPrev.item(2))
        self.assertAlmostEqual(beleif.v.item(0),vPrev.item(0))
        self.assertAlmostEqual(beleif.ba.item(0),baPrev.item(0))
        self.assertAlmostEqual(beleif.ba.item(1),baPrev.item(1))
        self.assertAlmostEqual(beleif.ba.item(2),baPrev.item(2))
        self.assertAlmostEqual(beleif.bg.item(0),bgPrev.item(0))
        self.assertAlmostEqual(beleif.bg.item(1),bgPrev.item(1))
        self.assertAlmostEqual(beleif.bg.item(2),bgPrev.item(2))

if __name__ == '__main__':
    unittest.main()