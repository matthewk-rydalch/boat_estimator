import numpy as np
from scipy.spatial.transform import Rotation as R
import navpy

from states import States

def propagate(xHat,Rt,dt):
     xHat.p = xHat.p + xHat.dp*dt
     xHat.q = xHat.q + xHat.dq*dt
     xHat.v = xHat.v + xHat.dv*dt
     xHat.ba = xHat.ba + xHat.dba*dt
     xHat.bg = xHat.bg + xHat.dbg*dt

     xHat.P = xHat.At@xHat.P@xHat.At.T + Rt

def update(xHat,Qt,zt,ht,Ct):
     Lt = xHat.P@Ct.T@np.linalg.inv(Ct@xHat.P@C.T+Qt)
     dx = Lt@(zt-ht)
     xHat.p = xHat.p + dx[0:3]
     xHat.q = xHat.q + dx[3:6]
     xHat.v = xHat.v + dx[6:9]
     xHat.ba = xHat.ba + dx[9:12]
     xHat.bg = xHat.bg + dx[12:15]

     xHat.P = (np.identity(15) - Lt@Ct)@xHat.P

def update_dynamic_model(xHat,ut,gravity,dt):
     accel = ut[0] - xHat.ba
     omega = ut[1] - xHat.bg
     Rb2v = R.from_rotvec(xHat.q.squeeze())
     Rv2b = Rb2v.inv()
     sphi = np.sin(xHat.q.item(0))
     cphi = np.cos(xHat.q.item(0))
     cth = np.cos(xHat.q.item(1))
     tth = np.tan(xHat.q.item(1))
     attitudeModelInversion = np.array([[1.0, sphi*tth, cphi*tth],
                                  [0.0, cphi, -sphi],
                                  [0.0, sphi/cth, cphi/cth]])

     xHat.dp = Rb2v.apply(xHat.v.T).T
     xHat.dq = attitudeModelInversion @ omega
     xHat.dv = accel + Rv2b.apply(gravity.T).T - np.cross(omega.T,xHat.v.T).T
     xHat.ba = np.array([[0.0,0.0,0.0]]).T
     xHat.bg = np.array([[0.0,0.0,0.0]]).T

def update_gps_measurement_model(xHat):
     h = np.concatenate((xHat.p,xHat.v),axis=0)
     return h

def update_compass_measurement_model(xHat):
     h = np.array([[xHat.q.item(2)]]).T
     return h

def update_Jacobian_A(xHat,omega):
     #TODO: check Jacobian more thouroughly
     g = np.array([[0.0,0.0,9.81]]).T
     spsi = np.sin(xHat.q.item(2))
     cpsi = np.cos(xHat.q.item(2))

     dpdp = np.zeros((3,3))
     dpdq = np.array([[spsi*xHat.v.item(2), cpsi*xHat.v.item(2), spsi*xHat.v.item(0)-cpsi*xHat.v.item(1)],
                      [0.0, spsi*xHat.v.item(2), cpsi*xHat.v.item(0)-spsi*xHat.v.item(1)],
                      [xHat.v.item(1),-xHat.v.item(0),0.0]])
     dpdv = R.from_rotvec(xHat.q.squeeze()).as_matrix()
     dpdBa = np.zeros((3,3))
     dpdBg = np.zeros((3,3))
     dpdx = np.concatenate((dpdp,dpdq,dpdv,dpdBa,dpdBg),axis=1)

     dqdp = np.zeros((3,3))
     dqdq = np.array([[0.0,omega.item(1),0.0],
                      [omega.item(2),0.0,0.0],
                      [omega.item(1),0.0,0.0]])
     dqdv = np.zeros((3,3))
     dqdBa = np.zeros((3,3))
     dqdBg = -np.identity(3)
     dqdx = np.concatenate((dqdp, dqdq, dqdv, dqdBa, dqdBg), axis=1)

     dvdp = np.zeros((3,3))
     dvdq = np.array([[spsi, cpsi, 0.0],
                      [spsi-cpsi, spsi, 0.0],
                      [0.0,0.0,0.0]])*g.item(2)
     dvdv = np.array([[0.0,-omega.item(2),omega.item(1)],
                      [omega.item(2), 0.0, -omega.item(0)],
                      [-omega.item(1),omega.item(0),0.0]])
     dvdBa = -np.identity(3)
     dvdBg = np.zeros((3,3))
     dvdx = np.concatenate((dvdp, dvdq, dvdv, dvdBa, dvdBg), axis=1)

     dBadp = np.zeros((3,3))
     dBadq = np.zeros((3,3))
     dBadv = np.zeros((3,3))
     dBadBa = np.zeros((3,3))
     dBadBg = np.zeros((3,3))
     dBadx = np.concatenate((dBadp, dBadq, dBadv, dBadBa, dBadBg), axis=1)

     dBgdp = np.zeros((3,3))
     dBgdq = np.zeros((3,3))
     dBgdv = np.zeros((3,3))
     dBgdBa = np.zeros((3,3))
     dBgdBg = np.zeros((3,3))
     dBgdx = np.concatenate((dBgdp, dBgdq, dBgdv, dBgdBa, dBgdBg), axis=1)

     xHat.At = np.concatenate((dpdx,dqdx,dvdx,dBadx,dBgdx),axis=0)

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
     

