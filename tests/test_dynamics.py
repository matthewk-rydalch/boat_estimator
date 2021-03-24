import sys
sys.path.append('../scripts')
import numpy as np

import unittest
import ekf
from states import States

class TestDynamics(unittest.TestCase):
    def test_no_inputs(self):
        p0 = np.zeros((3,1))
        q0 = np.zeros((3,1))
        v0 = np.zeros((3,1))
        ba0 = np.zeros((3,1))
        bg0 = np.zeros((3,1))
        cov0 = [1.0,1.0,1.0]
        xHat = States(p0,q0,v0,ba0,bg0,cov0,cov0,cov0,cov0,cov0)
        acclerometers = np.zeros((3,1))
        gyros = np.zeros((3,1))
        u = [acclerometers,gyros]
        dt = 0.1
        dpExpected = np.zeros((3,1))
        dqExpected = np.zeros((3,1))
        dvExpected = np.array([[0.0,0.0,9.81]]).T
        dbaExpected = np.zeros((3,1))
        dbgExpected = np.zeros((3,1))
        expectedDynamics = [dpExpected,dqExpected,dvExpected,dbaExpected,dbgExpected]
        ekf.update_dynamic_model(xHat,u,dt)
        self.assert_dynamics_equal(xHat,expectedDynamics)

    def test_inputs_equal_ones(self):
        p0 = np.zeros((3,1))
        q0 = np.zeros((3,1))
        v0 = np.zeros((3,1))
        ba0 = np.zeros((3,1))
        bg0 = np.zeros((3,1))
        cov0 = np.array([[1.0,1.0,1.0]]).T
        xHat = States(p0,q0,v0,ba0,bg0,cov0,cov0,cov0,cov0,cov0)
        accelerometers = np.array([[1.0,1.0,1.0]]).T
        gyros = np.array([[1.0,1.0,1.0]]).T
        u = [accelerometers,gyros]
        dt = 0.1
        dpExpected = np.zeros((3,1))
        dqExpected = np.array([[1.0,1.0,1.0]]).T
        dvExpected = np.array([[1.0,1.0,10.81]]).T
        dbaExpected = np.zeros((3,1))
        dbgExpected = np.zeros((3,1))
        expectedDynamics = [dpExpected,dqExpected,dvExpected,dbaExpected,dbgExpected]
        ekf.update_dynamic_model(xHat,u,dt)
        self.assert_dynamics_equal(xHat,expectedDynamics)

    def assert_dynamics_equal(self,xHat,expectedDynamics):
        for i in range(3):
            self.assertAlmostEqual(xHat.dp.item(i),expectedDynamics[0][i])
            self.assertAlmostEqual(xHat.dq.item(i),expectedDynamics[1][i])
            self.assertAlmostEqual(xHat.dv.item(i),expectedDynamics[2][i])
            self.assertAlmostEqual(xHat.dba.item(i),expectedDynamics[3][i])
            self.assertAlmostEqual(xHat.dbg.item(i),expectedDynamics[4][i])

if __name__ == '__main__':
    unittest.main()