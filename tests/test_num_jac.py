import sys
sys.path.append('/home/matt/px4_ws/src/boat_estimator/scripts/ekf')
sys.path.append('/home/matt/px4_ws/src/boat_estimator/scripts/structs')
import numpy as np
from scipy.spatial.transform import Rotation as R

import unittest
import ekf
from states_covariance import StatesCovariance

class TestNumericalJacobian(unittest.TestCase):
    def test_zeros(self):
        p = np.zeros((3,1))
        q = np.zeros((3,1))
        v = np.zeros((3,1))
        ba = np.zeros((3,1))
        bg = np.zeros((3,1))
        cov = np.identity(15)
        beleif = StatesCovariance(p,q,v,ba,bg,cov)
        accelerometers = np.array([[0.0,0.0,-9.81]]).T
        gyros = np.zeros((3,1))
        ut = [accelerometers,gyros]
        ft = ekf.update_dynamic_model
        At = ekf.calculate_numerical_jacobian(ft, beleif, ut)
        AtExpected = np.array([[ 0., 0., 0.,  0.      ,  0.      , 0.,  1., 0., 0.,  0.,  0.,  0.,  0., 0.,  0.],
                               [ 0., 0., 0.,  0.      ,  0.      , 0.,  0., 1., 0.,  0.,  0.,  0.,  0., 0.,  0.],
                               [ 0., 0., 0.,  0.      ,  0.      , 0.,  0., 0., 1.,  0.,  0.,  0.,  0., 0.,  0.],
                               [ 0., 0., 0.,  0.      ,  0.      , 0.,  0., 0., 0.,  0.,  0.,  0., -1., 0.,  0.],
                               [ 0., 0., 0.,  0.      ,  0.      , 0.,  0., 0., 0.,  0.,  0.,  0.,  0.,-1.,  0.],
                               [ 0., 0., 0.,  0.      ,  0.      , 0.,  0., 0., 0.,  0.,  0.,  0.,  0., 0., -1.],
                               [ 0., 0., 0.,  0.      , -9.809837, 0.,  0., 0., 0., -1.,  0.,  0.,  0., 0.,  0.],
                               [ 0., 0., 0.,  9.809837,  0.      , 0.,  0., 0., 0.,  0., -1.,  0.,  0., 0.,  0.],
                               [ 0., 0., 0., -0.04905 , -0.04905 , 0.,  0., 0., 0.,  0.,  0., -1.,  0., 0.,  0.],
                               [ 0., 0., 0.,  0.      ,  0.      , 0.,  0., 0., 0.,  0.,  0.,  0.,  0., 0.,  0.],
                               [ 0., 0., 0.,  0.      ,  0.      , 0.,  0., 0., 0.,  0.,  0.,  0.,  0., 0.,  0.],
                               [ 0., 0., 0.,  0.      ,  0.      , 0.,  0., 0., 0.,  0.,  0.,  0.,  0., 0.,  0.],
                               [ 0., 0., 0.,  0.      ,  0.      , 0.,  0., 0., 0.,  0.,  0.,  0.,  0., 0.,  0.],
                               [ 0., 0., 0.,  0.      ,  0.      , 0.,  0., 0., 0.,  0.,  0.,  0.,  0., 0.,  0.],
                               [ 0., 0., 0.,  0.      ,  0.      , 0.,  0., 0., 0.,  0.,  0.,  0.,  0., 0.,  0.]])
        self.assert_jacobian_equal(At,AtExpected)

    # def test_inputs_equal_ones(self):
    #     p = np.zeros((3,1))
    #     q = np.array([[0.1,0.1,0.1]]).T
    #     v = np.array([[0.1,0.1,1.081]]).T
    #     ba = np.zeros((3,1))
    #     bg = np.zeros((3,1))
    #     cov = np.array([[1.0,1.0,1.0]]).T
    #     beleif = StatesCovariance(p,q,v,ba,bg,cov,cov,cov,cov,cov)
    #     accelerometers = np.array([[1.0,1.0,1.0]]).T
    #     gyros = np.array([[1.0,1.0,1.0]]).T
    #     u = [accelerometers,gyros]
    #     dt = 0.1
    #     expectedA = np.zeros((15,15))
    #     expectedA[0:3,3:6] = [[np.sin(0.1)*1.081, np.cos(0.1)*1.081, np.sin(0.1)*(0.1)-np.cos(0.1)*0.1],
    #                           [0.0, np.sin(0.1)*1.081, np.cos(0.1)*0.1-np.sin(0.1)*0.1],
    #                           [0.1,-0.1,0.0]]
    #     expectedA[0:3,6:9] = R.from_rotvec([0.1,0.1,0.1]).as_matrix()
    #     expectedA[3:6,3:6] = [[0.0,1.0,0.0],
    #                           [1.0,0.0,0.0],
    #                           [1.0,0.0,0.0]]
    #     expectedA[3:6,12:15] = -np.identity(3)
    #     expectedA[6:9,3:6] = np.array([[np.sin(0.1), np.cos(0.1), 0.0],
    #                                    [np.sin(0.1)-np.cos(0.1),np.sin(0.1),0.0],
    #                                    [0.0,0.0,0.0]])*9.81
    #     expectedA[6:9,6:9] = [[0.0,-1.0,1.0],
    #                           [1.0,0.0,-1.0],
    #                           [-1.0,1.0,0.0]]
    #     expectedA[6:9,9:12] = -np.identity(3)
    #     At = ekf.update_Jacobian_A(beleif,u[1])
    #     self.assert_jacobian_equal(At,expectedA)

    def assert_jacobian_equal(self,At,expectedA):
        decimals = 3
        for j in range(15):
            for k in range(15):
                self.assertAlmostEqual(At[j][k],expectedA[j][k],decimals)

if __name__ == '__main__':
    unittest.main()