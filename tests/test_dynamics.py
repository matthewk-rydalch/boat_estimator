import sys
sys.path.append('/home/matt/px4_ws/src/boat_estimator/scripts/ekf')
sys.path.append('/home/matt/px4_ws/src/boat_estimator/scripts/structs')
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

    def test_random(self):
        p0 = np.array([[1.0,-3.2,4.4]]).T
        q0 = np.array([[2.3,2.2,-0.5]]).T
        v0 = np.array([[-2.1,1.1,0.2]]).T
        ba0 = np.zeros((3,1))
        bg0 = np.zeros((3,1))
        cov0 = np.array([[1.0,1.0,1.0]]).T
        beleif = StatesCovariance(p0,q0,v0,ba0,bg0,cov0,cov0,cov0,cov0,cov0)
        accelerometers = np.array([[-3.0,2.0,-1.0]]).T
        gyros = np.array([[4.0,-1.0,-1.0]]).T
        ft = DynamicModel()
        ut = [accelerometers,gyros]
        gravity = np.array([[0.0,0.0,9.81]]).T
        dt = 0.1
        dpExpected = [1.1491,-1.6329,1.2935]
        dqExpected = [4.1091,1.4120,0.1350]
        dvExpected = [-10.9313,-2.3051,2.8465]
        dbaExpected = [0.0,0.0,0.0]
        dbgExpected = [0.0,0.0,0.0]
        expectedDynamics = [dpExpected,dqExpected,dvExpected,dbaExpected,dbgExpected]
        ekf.update_dynamic_model(ft,beleif,ut,gravity,dt)
        self.assert_dynamics_equal(ft,expectedDynamics)

    def assert_dynamics_equal(self,ft,expectedDynamics):
        decimalPlaces = 3
        for i in range(3):
            self.assertAlmostEqual(ft.dp.item(i),expectedDynamics[0][i],decimalPlaces)
            self.assertAlmostEqual(ft.dq.item(i),expectedDynamics[1][i],decimalPlaces)
            self.assertAlmostEqual(ft.dv.item(i),expectedDynamics[2][i],decimalPlaces)
            self.assertAlmostEqual(ft.dba.item(i),expectedDynamics[3][i],decimalPlaces)
            self.assertAlmostEqual(ft.dbg.item(i),expectedDynamics[4][i],decimalPlaces)

if __name__ == '__main__':
    unittest.main()