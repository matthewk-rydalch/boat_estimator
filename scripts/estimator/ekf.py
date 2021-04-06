import numpy as np
from scipy.spatial.transform import Rotation as R

def propagate(belief,RProcess,RImu,ft,At,Bt,dt):
     belief.pr = belief.pr + ft.dpr*dt
     belief.vr = belief.vr + ft.dvr*dt
     belief.p = belief.p + ft.dp*dt
     # #belief.q[0:1] are estimated with the complementary filter
     belief.q[2] = belief.q[2] + ft.dq[2]*dt
     belief.v = belief.v + ft.dv*dt
     belief.ba = belief.ba + ft.dba*dt
     belief.bg = belief.bg + ft.dbg*dt

     Ad = np.identity(belief.m) + At*dt
     Bd = Bt*dt

     belief.P = Ad@belief.P@Ad.T + Bd@RImu@Bd.T + RProcess*dt**2

def update(belief,Qt,zt,ht,Ct):
     # print('Ct = ', Ct)
     # print('P = ', belief.P)
     # print('Qt = ', Qt)
     Lt = belief.P@Ct.T@np.linalg.inv(Ct@belief.P@Ct.T+Qt)
     dx = Lt@(zt-ht)
     belief.pr = belief.pr + dx[0:3]
     belief.vr = belief.vr + dx[3:6]
     belief.p = belief.p + dx[6:9]
     belief.q = belief.q + dx[9:12]
     belief.v = belief.v + dx[12:15]
     belief.ba = belief.ba + dx[15:18]
     belief.bg = belief.bg + dx[18:21]

     belief.P = (np.identity(belief.m) - Lt@Ct)@belief.P

def update_dynamic_model(belief,ut):
     gravity = np.array([[0.0,0.0,9.81]]).T
     accel = ut.accelerometers - belief.ba
     omega = ut.gyros - belief.bg

     Rb2i = R.from_euler('xyz',belief.q.squeeze())
     Ri2b = Rb2i.inv() #TODO q = nan some times cause it to break here
     sphi = np.sin(belief.q.item(0))
     cphi = np.cos(belief.q.item(0))
     cth = np.cos(belief.q.item(1))
     tth = np.tan(belief.q.item(1))
     attitudeModelInversion = np.array([[1.0, sphi*tth, cphi*tth],
                                  [0.0, cphi, -sphi],
                                  [0.0, sphi/cth, cphi/cth]])

     dpr = Rb2i.apply(belief.v.T).T - belief.vr
     dvr = np.zeros((3,1))
     dp = Rb2i.apply(belief.v.T).T
     dq = attitudeModelInversion @ omega
     dv = accel + Ri2b.apply(gravity.T).T - np.cross(omega.T,belief.v.T).T
     dba = np.zeros((3,1))
     dbg = np.zeros((3,1))
     ft = np.concatenate((dpr,dvr,dp,dq,dv,dba,dbg),axis=0)

     return ft

def update_relPos_measurement_model(belief):
     ht = -belief.pr
     return ht

def update_rover_velocity_measurement_model(belief):
     ht = belief.vr
     return ht

def update_base_gps_measurement_model(belief,gps):
     Rb2i = R.from_euler('xyz',belief.q.squeeze())
     beliefVelocityNed = Rb2i.apply(belief.v.T).T
     h = np.concatenate((belief.p,beliefVelocityNed),axis=0)
     return h

def update_compass_measurement_model(belief):
     h = np.array([[belief.q.item(2)]]).T
     return h
    
def update_jacobian_B(belief):
     sphi = np.sin(belief.q.item(0))
     cphi = np.cos(belief.q.item(0))
     cth = np.cos(belief.q.item(1))
     tth = np.tan(belief.q.item(1))
     attitudeModelInversion = np.array([[1.0, sphi * tth, cphi * tth],
                                        [0.0, cphi, -sphi],
                                        [0.0, sphi / cth, cphi / cth]])

     dprda = np.zeros((3,3))
     dprdw = np.zeros((3,3))
     dprdu = np.concatenate((dprda, dprdw),axis=1)

     dvrda = np.zeros((3,3))
     dvrdw = np.zeros((3,3))
     dvrdu = np.concatenate((dvrda, dvrdw),axis=1)

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

     Bt = np.concatenate((dprdu, dvrdu, dpdu, dqdu, dvdu, dBadu, dBgdu), axis=0)

     return Bt

def get_jacobian_C_relPos():
     dprdpr = -np.identity(3)
     dprdvr = np.zeros((3,3))
     dprdp = np.zeros((3,3))
     dprdq = np.zeros((3,3))
     dprdv = np.zeros((3,3))
     dprdBa = np.zeros((3,3))
     dprdBg = np.zeros((3,3))
     Ct = np.concatenate((dprdpr,dprdvr,dprdp,dprdq,dprdv,dprdBa,dprdBg),axis=1)
     
     return Ct

def get_jacobian_C_rover_velocity():
     dvrdpr = np.zeros((3,3))
     dvrdvr = np.identity(3)
     dvrdp = np.zeros((3,3))
     dvrdq = np.zeros((3,3))
     dvrdv = np.zeros((3,3))
     dvrdBa = np.zeros((3,3))
     dvrdBg = np.zeros((3,3))
     Ct = np.concatenate((dvrdpr,dvrdvr,dvrdp,dvrdq,dvrdv,dvrdBa,dvrdBg),axis=1)
     
     return Ct

def get_jacobian_C_compass():
     dpsidpr = np.zeros((1,3))
     dpsidvr = np.zeros((1,3))
     dpsidp = np.zeros((1,3))
     dpsidq = np.array([[0.0,0.0,1.0]])
     dpsidv = np.zeros((1,3))
     dpsidba = np.zeros((1,3))
     dpsidbg = np.zeros((1,3))
     Ct = np.concatenate((dpsidpr,dpsidvr,dpsidp,dpsidq,dpsidv,dpsidba,dpsidbg),axis=1)

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