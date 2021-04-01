import sys
sys.path.append('/home/matt/px4_ws/src/boat_estimator/scripts/ekf')
sys.path.append('/home/matt/px4_ws/src/boat_estimator/scripts/structs')
import numpy as np

import unittest
import ekf
from dynamic_model import DynamicModel
from states_covariance import StatesCovariance

class TestPropagation(unittest.TestCase):
    def test_zeros(self):
        p0 = np.zeros((3,1))
        q0 = np.zeros((3,1))
        v0 = np.zeros((3,1))
        ba0 = np.zeros((3,1))
        bg0 = np.zeros((3,1))
        cov0 = np.identity(15)
        beleif = StatesCovariance(p0, q0, v0, ba0, bg0, cov0)

        RProcess = np.zeros((15, 15))
        RImu = np.zeros((6, 6))

        dp = np.zeros((3,1))
        dq = np.zeros((3,1))
        dv = np.zeros((3,1))
        dba = np.zeros((3,1))
        dbg = np.zeros((3,1))
        dynamics = np.concatenate((dp, dq, dv, dba, dbg), axis=0)
        ft = DynamicModel(dynamics)

        At = np.zeros((15,15))
        Bt = np.zeros((15, 6))

        dt = 0.1
        ekf.propagate(beleif, RProcess, RImu, ft, At, Bt, dt)

        pExpected = np.zeros((3,1))
        qExpected = np.zeros((3,1))
        vExpected = np.zeros((3,1))
        baExpected = np.zeros((3,1))
        bgExpected = np.zeros((3,1))
        beleifExpected = StatesCovariance(pExpected, qExpected, vExpected, baExpected, bgExpected, cov0)

        self.assert_states_equal(beleif, beleifExpected)
        self.assert_covariances_equal(beleif.P, beleifExpected.P)

    def test_state_propagation(self):
        p0 = np.array([[0.1,-0.2,4.0]]).T
        q0 = np.array([[-0.0,-2.2,3.0]]).T
        v0 = np.array([[2.2,4.1,2.2]]).T
        ba0 = np.array([[0.1,2.1,-1.0]]).T
        bg0 = np.array([[8.4,2.1,-2.0]]).T
        cov0 = np.identity(15)
        beleif = StatesCovariance(p0,q0,v0,ba0,bg0,cov0)

        RProcess = np.zeros((15,15))
        RImu = np.zeros((6,6))

        dp = np.array([[1.2,-0.3,0.0]]).T
        dq = np.array([[-0.1,-2.1,1.0]]).T
        dv = np.array([[3.4,1.1,2.0]]).T
        dba = np.array([[4.3,0.0,-0.0]]).T
        dbg = np.array([[3.2,3.0,0.22]]).T
        dynamics = np.concatenate((dp,dq,dv,dba,dbg),axis=0)
        ft = DynamicModel(dynamics)

        At = np.identity(15)
        Bt = np.zeros((15,6))

        dt = 0.1
        ekf.propagate(beleif,RProcess,RImu,ft,At,Bt,dt)

        pExpected = np.array([[0.22,-0.23,4.0]]).T
        qExpected = np.array([[-0.01,-2.41,3.1]]).T
        vExpected = np.array([[2.54,4.21,2.4]]).T
        baExpected = np.array([[0.53,2.1,-1.0]]).T
        bgExpected = np.array([[8.72,2.4,-1.978]]).T
        beleifExpected = StatesCovariance(pExpected,qExpected,vExpected,baExpected,bgExpected,cov0,cov0,cov0,cov0,cov0)

        self.assert_states_equal(beleif,beleifExpected)

    def test_covariance_propagation(self):
        ####Not Important Place Holders####
        p0 = np.zeros((3,1))
        q0 = np.zeros((3,1))
        v0 = np.zeros((3,1))
        ba0 = np.zeros((3,1))
        bg0 = np.zeros((3,1))
        cov0 = np.identity(15)
        beleif = StatesCovariance(p0, q0, v0, ba0, bg0, cov0)
        dp = np.zeros((3,1))
        dq = np.zeros((3,1))
        dv = np.zeros((3,1))
        dba = np.zeros((3,1))
        dbg = np.zeros((3,1))
        dynamics = np.concatenate((dp,dq,dv,dba,dbg),axis=0)
        ft = DynamicModel(dynamics)
        ####

        RProcess = np.array([[1.0,2.0,3.0],
                             [0.0,1.0,-1.0],
                             [1.0,1.0,-2.0]])
        RImu = np.array([[0.2,0.1,0.3],
                         [0.4,1.0,0.4],
                         [0.1,2.0,1.1]])

        At = np.array([[1.0,-1.0,1.0],
                      [2.0,2.0,0.0],
                      [0.0,1.0,1.0]])
        Bt = np.array([[1.2,-1.0,0.2],
                      [2.0,0.0,0.1],
                      [0.0,2.0,1.0]])
        P = np.array([[2.0,1.0,0.0],
                      [0.0,2.0,-1.0],
                      [1.0,0.0,-1.0]])
        beleif.P = P

        dt = 0.1
        ekf.propagate(beleif,RProcess,RImu,ft,At,Bt,dt)

        PExpected = np.array([[2.45348,1.55738,0.1122],
                              [0.06234,3.21891,-1.0549],
                              [1.0546,0.4899,-1.221]])

        self.assert_states_equal(beleif.P,PExpected)

    # def test_covariance_propagation(self):
    #     dt = 0.1
    #     Rt = np.array([[1.0,2.0,3.0],
    #                    [0.0,1.0,-1.0],
    #                    [1.0,1.0,-2.0]])
    #     At = np.array([[1.0,-1.0,1.0],
    #                   [2.0,2.0,0.0],
    #                   [0.0,1.0,1.0]])
    #     P = np.array([[2.0,1.0,0.0],
    #                   [0.0,2.0,-1.0],
    #                   [1.0,0.0,-1.0]])
    #     p0 = np.zeros((3,1))
    #     q0 = np.zeros((3,1))
    #     v0 = np.zeros((3,1))
    #     ba0 = np.zeros((3,1))
    #     bg0 = np.zeros((3,1))
    #     cov0 = np.array([[1.0,1.0,1.0]]).T
    #     beleif = StatesCovariance(p0,q0,v0,ba0,bg0,cov0,cov0,cov0,cov0,cov0)
    #     beleif.P = P
    #     ft = DynamicModel()
    #     ekf.propagate(beleif,Rt,ft,At,dt)
    #     expectedP = np.array([[5.0,6.0,2.0],
    #                           [-4.0,21.0,3.0],
    #                           [-2.0,7.0,-2.0]])
    #     self.assert_covariances_equal(beleif,expectedP)

    def assert_states_equal(self,beleif,beleifExpected):
        for i in range(3):
            self.assertAlmostEqual(beleif.p.item(i),beleifExpected.p.item(i))
            self.assertAlmostEqual(beleif.q.item(i),beleifExpected.q.item(i))
            self.assertAlmostEqual(beleif.v.item(i),beleifExpected.v.item(i))
            self.assertAlmostEqual(beleif.ba.item(i),beleifExpected.ba.item(i))
            self.assertAlmostEqual(beleif.bg.item(i),beleifExpected.bg.item(i))

    def assert_covariances_equal(self,P,expectedP):
        for i in range(3):
            for j in range(3):
                self.assertAlmostEqual(P[i][j],expectedP[i][j])


if __name__ == '__main__':
    unittest.main()