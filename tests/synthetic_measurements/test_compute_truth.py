import sys
sys.path.append('/home/matt/px4_ws/src/boat_estimator/scripts/misc')
sys.path.append('/home/matt/px4_ws/src/boat_estimator/scripts/structs')
import numpy as np

import unittest
import synthetic_measurements
from sensors import TruthMsg

class TestDynamics(unittest.TestCase):
    def test_compute_truth(self):
        print('This test depends on what models are set in the function')
        t = np.pi/2.0
        truth = TruthMsg()
        position = [-3.0,1.0,-1.0]
        orientation = [0.2,0.3,-0.1]
        velocity = [-2.96377917,  1.48202406,  0.14006332]
        angularVelocity = [0.0,0.0,0.0]
        acceleration = [0.0,0.0,0.0]
        expectedTruth = TruthMsg(t,position,orientation,velocity,angularVelocity,acceleration)
        synthetic_measurements.compute_truth(t,truth)
        self.assert_truths(truth,expectedTruth)

    def assert_truths(self,truth,expectedTruth):
        for i in range(3):
            self.assertAlmostEqual(truth.position.item(i),expectedTruth.position.item(i))
            self.assertAlmostEqual(truth.orientation.item(i),expectedTruth.orientation.item(i))
            self.assertAlmostEqual(truth.velocity.item(i),expectedTruth.velocity.item(i))
            self.assertAlmostEqual(truth.angularVelocity.item(i),expectedTruth.angularVelocity.item(i))
            self.assertAlmostEqual(truth.acceleration.item(i),expectedTruth.acceleration.item(i))
 
if __name__ == '__main__':
    unittest.main()