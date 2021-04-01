import numpy as np
from scipy.spatial.transform import Rotation as R
import navpy

import sys
sys.path.append('/home/matt/px4_ws/src/boat_estimator/src/structs')

from states_covariance import StatesCovariance

def propagate(beleif,RProcess,RImu,ft,At,Bt,dt):
     beleif.p = beleif.p + ft.dp*dt
     beleif.q[2] = beleif.q[2] + ft.dq[2]*dt
     beleif.v = beleif.v + ft.dv*dt
     beleif.ba = beleif.ba + ft.dba*dt
     beleif.bg = beleif.bg + ft.dbg*dt

     Ad = np.identity(15) + At*dt
     Bd = Bt*dt
     beleif.P = Ad@beleif.P@Ad.T + Bd@RImu@Bd.T + RProcess*dt**2

def update(beleif,Qt,zt,ht,Ct):
     Lt = beleif.P@Ct.T@np.linalg.inv(Ct@beleif.P@Ct.T+Qt)
     dx = Lt@(zt-ht)
     beleif.p = beleif.p + dx[0:3]
     beleif.q = beleif.q + dx[3:6]
     beleif.v = beleif.v + dx[6:9]
     beleif.ba = beleif.ba + dx[9:12]
     beleif.bg = beleif.bg + dx[12:15]

     print('beleif.ba = ', beleif.ba)
     print('beleif.bg = ', beleif.bg)

     beleif.P = (np.identity(15) - Lt@Ct)@beleif.P

def update_dynamic_model(beleif,ut):
     #TODO fix belief spelling everywhere
     accel = ut.accelerometers - beleif.ba
     omega = ut.gyros - beleif.bg
     Rb2i = R.from_euler('xyz',beleif.q.squeeze())
     Ri2b = Rb2i.inv()
     sphi = np.sin(beleif.q.item(0))
     cphi = np.cos(beleif.q.item(0))
     cth = np.cos(beleif.q.item(1))
     tth = np.tan(beleif.q.item(1))
     attitudeModelInversion = np.array([[1.0, sphi*tth, cphi*tth],
                                  [0.0, cphi, -sphi],
                                  [0.0, sphi/cth, cphi/cth]])
     dp = Rb2i.apply(beleif.v.T).T
     dq = attitudeModelInversion @ omega
     dv = accel + Ri2b.apply(np.array([[0.0,0.0,9.81]])).T - np.cross(omega.T,beleif.v.T).T
     dba = np.array([[0.0,0.0,0.0]]).T
     dbg = np.array([[0.0,0.0,0.0]]).T
     ft = np.concatenate((dp,dq,dv,dba,dbg),axis=0)

     return ft

def update_gps_measurement_model(beleif):
     h = np.concatenate((beleif.p,beleif.v),axis=0)
     return h

def update_compass_measurement_model(beleif):
     h = np.array([[beleif.q.item(2)]]).T
     return h

def calculate_numerical_jacobian_A(fun, xt, ut):
    ft = fun(xt, ut)
    m = len(ft)
    n = xt.m
    eps = 0.01
    J = np.zeros((m, n))
    test = np.zeros((15,0))
    for i in range(0, n):
        x_eps = xt.get_copy()
        x_eps[i] = x_eps[i] + eps
        f_eps = fun(x_eps, ut)
        test = np.concatenate((test,f_eps),axis=1)
        df = (f_eps - ft) / eps
        J[:, i] = df[:, 0]
    return J

def update_jacobian_B(beleif):
    sphi = np.sin(beleif.q.item(0))
    cphi = np.cos(beleif.q.item(0))
    cth = np.cos(beleif.q.item(1))
    tth = np.tan(beleif.q.item(1))
    attitudeModelInversion = np.array([[1.0, sphi * tth, cphi * tth],
                                       [0.0, cphi, -sphi],
                                       [0.0, sphi / cth, cphi / cth]])

    dpda = np.zeros((3, 3))
    dpdw = np.zeros((3, 3))
    dpdu = np.concatenate((dpda, dpdw), axis=1)

    dqda = np.zeros((3, 3))
    dqdw = attitudeModelInversion
    dqdu = np.concatenate((dqda, dqdw), axis=1)

    dvda = np.identity(3)
    dvdw = np.array([[0.0, beleif.v.item(2), -beleif.v.item(1)],
                     [-beleif.v.item(2), 0.0, beleif.v.item(0)],
                     [beleif.v.item(1), -beleif.v.item(0), 0.0]])
    dvdu = np.concatenate((dvda, dvdw), axis=1)

    dBada = np.identity(3)
    dBadw = np.identity(3)
    dBadu = np.concatenate((dBada, dBadw), axis=1)

    dBgda = np.identity(3)
    dBgdw = np.identity(3)
    dBgdu = np.concatenate((dBgda, dBgdw), axis=1)

    Bt = np.concatenate((dpdu, dqdu, dvdu, dBadu, dBgdu), axis=0)

    return Bt

def get_jacobian_C_gps():
     dpdp = np.identity(3)
     dpdq = np.zeros((3,3))
     dpdv = np.zeros((3,3))
     dpdba = np.zeros((3,3))
     dpdbg = np.zeros((3,3))
     dpdx = np.concatenate((dpdp,dpdq,dpdv,dpdba,dpdbg),axis=1)

     dvdp = np.zeros((3,3))
     dvdq = np.zeros((3,3))
     dvdv = np.identity(3)
     dvdba = np.zeros((3,3))
     dvdbg = np.zeros((3,3))
     dvdx = np.concatenate((dvdp, dvdq, dvdv, dvdba, dvdbg), axis=1)

     Ct = np.concatenate((dpdx,dvdx),axis=0)
     
     return Ct

def get_jacobian_C_compass():
     dpsidp = np.zeros((1,3))
     dpsidq = np.array([[0.0,0.0,1.0]])
     dpsidv = np.zeros((1,3))
     dpsidba = np.zeros((1,3))
     dpsidbg = np.zeros((1,3))
     Ct = np.concatenate((dpsidp,dpsidq,dpsidv,dpsidba,dpsidbg),axis=1)

     return Ct

def get_skew_symetric_matrix(v):
     m = np.array([[0.0,-v.item(2),v.item(1)],
                   [v.item(2),0.0,-v.item(0)],
                   [-v.item(1),v.item(0),0.0]])
     return m
     

