import numpy as np
from scipy.spatial.transform import Rotation as R

def propagate(belief,RProcess,RImu,ft,At,Bt,dt):
     belief.p = belief.p + ft.dp*dt
     #belief.q[0:1] are estimated with the complementary filter
     belief.q[2] = belief.q[2] + ft.dq[2]*dt
     belief.v = belief.v + ft.dv*dt
     belief.ba = belief.ba + ft.dba*dt
     belief.bg = belief.bg + ft.dbg*dt

     Ad = np.identity(15) + At*dt
     Bd = Bt*dt

     print('belief.ba = ', belief.ba)
     belief.P = Ad@belief.P@Ad.T + Bd@RImu@Bd.T + RProcess*dt**2

def update(belief,Qt,zt,ht,Ct):
     #TODO:Figure out why this sometimes breaks.  Print statements and rosbags seem to break it.  It is fairly random
     Lt = belief.P@Ct.T@np.linalg.inv(Ct@belief.P@Ct.T+Qt)
     dx = Lt@(zt-ht)
     belief.p = belief.p + dx[0:3]
     belief.q = belief.q + dx[3:6]
     belief.v = belief.v + dx[6:9]
     belief.ba = belief.ba + dx[9:12]
     belief.bg = belief.bg + dx[12:15]

     belief.P = (np.identity(15) - Lt@Ct)@belief.P

def update_dynamic_model(belief,ut):
     gravity = np.array([[0.0,0.0,9.81]]).T
     accel = ut.accelerometers - belief.ba
     omega = ut.gyros - belief.bg

     Rb2i = R.from_euler('xyz',belief.q.squeeze())
     Ri2b = Rb2i.inv()
     sphi = np.sin(belief.q.item(0))
     cphi = np.cos(belief.q.item(0))
     cth = np.cos(belief.q.item(1))
     tth = np.tan(belief.q.item(1))
     attitudeModelInversion = np.array([[1.0, sphi*tth, cphi*tth],
                                  [0.0, cphi, -sphi],
                                  [0.0, sphi/cth, cphi/cth]])

     dp = Rb2i.apply(belief.v.T).T
     dq = attitudeModelInversion @ omega
     dv = accel + Ri2b.apply(gravity.T).T - np.cross(omega.T,belief.v.T).T
     dba = np.array([[0.0,0.0,0.0]]).T
     dbg = np.array([[0.0,0.0,0.0]]).T
     ft = np.concatenate((dp,dq,dv,dba,dbg),axis=0)

     return ft

def update_gps_measurement_model(belief,gps):
     h = np.concatenate((belief.p,belief.v),axis=0)
     return h

def update_compass_measurement_model(belief):
     h = np.array([[belief.q.item(2)]]).T
     return h

def calculate_numerical_jacobian_A(get_dynamics_function, xt, ut):
    ft = get_dynamics_function(xt, ut)
    m = len(ft)
    n = xt.m
    epsilon = 0.01
    J = np.zeros((m, n))
    test = np.zeros((15,0))
    for i in range(0, n):
        xkPlusOne = xt.get_copy()
        xkPlusOne.add_to_item(i,epsilon)
        fkPlusOne = get_dynamics_function(xkPlusOne, ut)
        test = np.concatenate((test,fkPlusOne),axis=1)
        df = (fkPlusOne - ft) / epsilon
        J[:, i] = df[:, 0]
    return J

def update_jacobian_B(belief):
    sphi = np.sin(belief.q.item(0))
    cphi = np.cos(belief.q.item(0))
    cth = np.cos(belief.q.item(1))
    tth = np.tan(belief.q.item(1))
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
    dvdw = np.array([[0.0, belief.v.item(2), -belief.v.item(1)],
                     [-belief.v.item(2), 0.0, belief.v.item(0)],
                     [belief.v.item(1), -belief.v.item(0), 0.0]])
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

def get_numerical_jacobian(fun, xk, zk):
    fk = fun(xk, zk)
    m = fk.shape[0]
    n = xk.m
    epsilon = 0.01
    J = np.zeros((m, n))
    for i in range(0, n):
        xkPlusOne = xk.get_copy()
        xkPlusOne.add_to_item(i,epsilon)
        fkPlusOne = fun(xkPlusOne, zk)
        dfdx = (fkPlusOne - fk) / epsilon
        J[:, i] = dfdx[:, 0]
    return J