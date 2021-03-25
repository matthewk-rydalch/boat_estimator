import sys
sys.path.append('/home/matt/px4_ws/src/boat_estimator/scripts')
import numpy as np

import unittest
import ekf
from dynamic_model import DynamicModel
from states_covariance import StatesCovariance

class TestDynamics(unittest.TestCase):
    def test_no_inputs(self):
        p0 = np.zeros((3,1))
        q0 = np.zeros((3,1))
        v0 = np.zeros((3,1))
        ba0 = np.zeros((3,1))
        bg0 = np.zeros((3,1))
        cov0 = np.array([[1.0,1.0,1.0]]).T
        beleif = StatesCovariance(p0,q0,v0,ba0,bg0,cov0,cov0,cov0,cov0,cov0)
        acclerometers = np.zeros((3,1))
        gyros = np.zeros((3,1))
        ft = DynamicModel()
        ut = [acclerometers,gyros]
        gravity = np.array([[0.0,0.0,9.81]]).T
        dt = 0.1
        dpExpected = np.zeros((3,1))
        dqExpected = np.zeros((3,1))
        dvExpected = np.array([[0.0,0.0,9.81]]).T
        dbaExpected = np.zeros((3,1))
        dbgExpected = np.zeros((3,1))
        expectedDynamics = [dpExpected,dqExpected,dvExpected,dbaExpected,dbgExpected]
        ekf.update_dynamic_model(ft,beleif,ut,gravity,dt)
        self.assert_dynamics_equal(ft,expectedDynamics)

    def test_inputs_equal_ones(self):
        p0 = np.zeros((3,1))
        q0 = np.zeros((3,1))
        v0 = np.zeros((3,1))
        ba0 = np.zeros((3,1))
        bg0 = np.zeros((3,1))
        cov0 = np.array([[1.0,1.0,1.0]]).T
        beleif = StatesCovariance(p0,q0,v0,ba0,bg0,cov0,cov0,cov0,cov0,cov0)
        accelerometers = np.array([[1.0,1.0,1.0]]).T
        gyros = np.array([[1.0,1.0,1.0]]).T
        ft = DynamicModel()
        ut = [accelerometers,gyros]
        gravity = np.array([[0.0,0.0,9.81]]).T
        dt = 0.1
        dpExpected = np.zeros((3,1))
        dqExpected = np.array([[1.0,1.0,1.0]]).T
        dvExpected = np.array([[1.0,1.0,10.81]]).T
        dbaExpected = np.zeros((3,1))
        dbgExpected = np.zeros((3,1))
        expectedDynamics = [dpExpected,dqExpected,dvExpected,dbaExpected,dbgExpected]
        ekf.update_dynamic_model(ft,beleif,ut,gravity,dt)
        self.assert_dynamics_equal(ft,expectedDynamics)

    def assert_dynamics_equal(self,ft,expectedDynamics):
        for i in range(3):
            self.assertAlmostEqual(ft.dp.item(i),expectedDynamics[0][i])
            self.assertAlmostEqual(ft.dq.item(i),expectedDynamics[1][i])
            self.assertAlmostEqual(ft.dv.item(i),expectedDynamics[2][i])
            self.assertAlmostEqual(ft.dba.item(i),expectedDynamics[3][i])
            self.assertAlmostEqual(ft.dbg.item(i),expectedDynamics[4][i])

if __name__ == '__main__':
    unittest.main()