import sys
sys.path.append('/home/matt/px4_ws/src/boat_estimator/scripts')
import numpy as np

import unittest
import ekf
from states import States
from ekf_class import EKF

class TestPredictionStep(unittest.TestCase):
    def test_acceleration(self):
        accelerometers = np.array([[1.0,-1.0,-9.0]]).T
        gyros = np.array([[0.0,0.0,0.0]]).T
        u = [accelerometers,gyros]
        dt = 0.1
        estimator = EKF()
        steps = 5
        for i in range(steps):
            pPrev = estimator.xHat.p
            qPrev = estimator.xHat.q
            vPrev = estimator.xHat.v
            baPrev = estimator.xHat.ba
            bgPrev = estimator.xHat.bg
            estimator.run_prediction_step(u,dt)
            self.check_acceleration_trend(estimator.xHat,pPrev,qPrev,vPrev,baPrev,bgPrev)

    def test_yaw(self):
        accelerometers = np.array([[0.0,0.0,-9.81]]).T
        gyros = np.array([[0.0,0.0,1.0]]).T
        u = [accelerometers,gyros]
        dt = 0.1
        estimator = EKF()
        steps = 5
        for i in range(steps):
            pPrev = estimator.xHat.p
            qPrev = estimator.xHat.q
            vPrev = estimator.xHat.v
            baPrev = estimator.xHat.ba
            bgPrev = estimator.xHat.bg
            estimator.run_prediction_step(u,dt)
            self.check_yaw_trend(estimator.xHat,pPrev,qPrev,vPrev,baPrev,bgPrev)

    def test_pitch_xz_acceleration(self):
        accelerometers = np.array([[0.0,0.0,-9.81]]).T
        gyros = np.array([[0.0,1.0,0.0]]).T
        u = [accelerometers,gyros]
        dt = 0.1
        estimator = EKF()
        steps = 5
        for i in range(steps):
            pPrev = estimator.xHat.p
            qPrev = estimator.xHat.q
            vPrev = estimator.xHat.v
            baPrev = estimator.xHat.ba
            bgPrev = estimator.xHat.bg
            estimator.run_prediction_step(u,dt)
            self.check_pitch_trend(estimator.xHat,pPrev,qPrev,vPrev,baPrev,bgPrev)

    def test_roll_yz_acceleration(self):
        accelerometers = np.array([[0.0,0.0,-9.81]]).T
        gyros = np.array([[1.0,0.0,0.0]]).T
        u = [accelerometers,gyros]
        dt = 0.1
        estimator = EKF()
        steps = 5
        for i in range(steps):
            pPrev = estimator.xHat.p
            qPrev = estimator.xHat.q
            vPrev = estimator.xHat.v
            baPrev = estimator.xHat.ba
            bgPrev = estimator.xHat.bg
            estimator.run_prediction_step(u,dt)
            self.check_roll_trend(estimator.xHat,pPrev,qPrev,vPrev,baPrev,bgPrev)

    def check_acceleration_trend(self,xHat,pPrev,qPrev,vPrev,baPrev,bgPrev):
        self.assertGreaterEqual(xHat.p.item(0),pPrev.item(0))
        self.assertGreaterEqual(xHat.p.item(2),pPrev.item(2))

        self.assertGreater(xHat.v.item(0),vPrev.item(0))
        self.assertGreater(xHat.v.item(2),vPrev.item(2))

        self.assertLessEqual(xHat.p.item(1),pPrev.item(1))

        self.assertLess(xHat.v.item(1),vPrev.item(1))

        self.assertAlmostEqual(xHat.q.item(0),qPrev.item(0))
        self.assertAlmostEqual(xHat.q.item(1),qPrev.item(1))
        self.assertAlmostEqual(xHat.q.item(2),qPrev.item(2))
        self.assertAlmostEqual(xHat.ba.item(0),baPrev.item(0))
        self.assertAlmostEqual(xHat.ba.item(1),baPrev.item(1))
        self.assertAlmostEqual(xHat.ba.item(2),baPrev.item(2))
        self.assertAlmostEqual(xHat.bg.item(0),bgPrev.item(0))
        self.assertAlmostEqual(xHat.bg.item(1),bgPrev.item(1))
        self.assertAlmostEqual(xHat.bg.item(2),bgPrev.item(2))

    def check_yaw_trend(self,xHat,pPrev,qPrev,vPrev,baPrev,bgPrev):
        self.assertGreater(xHat.q.item(2),qPrev.item(2))

        self.assertAlmostEqual(xHat.p.item(0),pPrev.item(0))
        self.assertAlmostEqual(xHat.p.item(1),pPrev.item(1))
        self.assertAlmostEqual(xHat.p.item(2),pPrev.item(2))
        self.assertAlmostEqual(xHat.q.item(0),qPrev.item(0))
        self.assertAlmostEqual(xHat.q.item(1),qPrev.item(1))
        self.assertAlmostEqual(xHat.v.item(0),vPrev.item(0))
        self.assertAlmostEqual(xHat.v.item(1),vPrev.item(1))
        self.assertAlmostEqual(xHat.v.item(2),vPrev.item(2))
        self.assertAlmostEqual(xHat.ba.item(0),baPrev.item(0))
        self.assertAlmostEqual(xHat.ba.item(1),baPrev.item(1))
        self.assertAlmostEqual(xHat.ba.item(2),baPrev.item(2))
        self.assertAlmostEqual(xHat.bg.item(0),bgPrev.item(0))
        self.assertAlmostEqual(xHat.bg.item(1),bgPrev.item(1))
        self.assertAlmostEqual(xHat.bg.item(2),bgPrev.item(2))


    def check_pitch_trend(self,xHat,pPrev,qPrev,vPrev,baPrev,bgPrev):
        self.assertGreaterEqual(xHat.p.item(2),pPrev.item(2))

        self.assertGreater(xHat.q.item(1),qPrev.item(1))

        self.assertLessEqual(xHat.p.item(0),pPrev.item(0))
        self.assertLessEqual(xHat.v.item(0),vPrev.item(0))
        self.assertLessEqual(xHat.v.item(2),vPrev.item(2))

        self.assertAlmostEqual(xHat.p.item(1),pPrev.item(1))
        self.assertAlmostEqual(xHat.q.item(0),qPrev.item(0))
        self.assertAlmostEqual(xHat.q.item(2),qPrev.item(2))
        self.assertAlmostEqual(xHat.v.item(1),vPrev.item(1))
        self.assertAlmostEqual(xHat.ba.item(0),baPrev.item(0))
        self.assertAlmostEqual(xHat.ba.item(1),baPrev.item(1))
        self.assertAlmostEqual(xHat.ba.item(2),baPrev.item(2))
        self.assertAlmostEqual(xHat.bg.item(0),bgPrev.item(0))
        self.assertAlmostEqual(xHat.bg.item(1),bgPrev.item(1))
        self.assertAlmostEqual(xHat.bg.item(2),bgPrev.item(2))

    def check_roll_trend(self,xHat,pPrev,qPrev,vPrev,baPrev,bgPrev):
        self.assertGreaterEqual(xHat.p.item(1),pPrev.item(1))
        self.assertGreaterEqual(xHat.p.item(2),pPrev.item(2))
        self.assertGreaterEqual(xHat.v.item(1),vPrev.item(1))

        self.assertGreater(xHat.q.item(0),qPrev.item(0))

        self.assertLessEqual(xHat.v.item(2),vPrev.item(2))

        self.assertAlmostEqual(xHat.p.item(0),pPrev.item(0))
        self.assertAlmostEqual(xHat.q.item(1),qPrev.item(1))
        self.assertAlmostEqual(xHat.q.item(2),qPrev.item(2))
        self.assertAlmostEqual(xHat.v.item(0),vPrev.item(0))
        self.assertAlmostEqual(xHat.ba.item(0),baPrev.item(0))
        self.assertAlmostEqual(xHat.ba.item(1),baPrev.item(1))
        self.assertAlmostEqual(xHat.ba.item(2),baPrev.item(2))
        self.assertAlmostEqual(xHat.bg.item(0),bgPrev.item(0))
        self.assertAlmostEqual(xHat.bg.item(1),bgPrev.item(1))
        self.assertAlmostEqual(xHat.bg.item(2),bgPrev.item(2))

if __name__ == '__main__':
    unittest.main()