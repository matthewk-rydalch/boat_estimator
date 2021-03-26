import numpy as np
from scipy.spatial.transform import Rotation as R
import navpy

import sys
sys.path.append('/home/matt/px4_ws/src/boat_estimator/src/structs')

from states_covariance import StatesCovariance

def propagate(beleif,Rt,ft,At,dt):
     beleif.p = beleif.p + ft.dp*dt
     beleif.q = beleif.q + ft.dq*dt
     beleif.v = beleif.v + ft.dv*dt
     beleif.ba = beleif.ba + ft.dba*dt
     beleif.bg = beleif.bg + ft.dbg*dt

     beleif.P = At@beleif.P@At.T + Rt

def update(beleif,Qt,zt,ht,Ct):
     Lt = beleif.P@Ct.T@np.linalg.inv(Ct@beleif.P@Ct.T+Qt)
     dx = Lt@(zt-ht)
     beleif.p = beleif.p + dx[0:3]
     beleif.q = beleif.q + dx[3:6]
     beleif.v = beleif.v + dx[6:9]
     beleif.ba = beleif.ba + dx[9:12]
     beleif.bg = beleif.bg + dx[12:15]

     beleif.P = (np.identity(15) - Lt@Ct)@beleif.P

def update_dynamic_model(ft,beleif,ut,gravity,dt):
     accel = ut[0] - beleif.ba
     omega = ut[1] - beleif.bg
     print('omega = ', omega)
     print('beleif q = ', beleif.q)
     Rb2v = R.from_rotvec(beleif.q.squeeze())
     Rv2b = Rb2v.inv()
     sphi = np.sin(beleif.q.item(0))
     cphi = np.cos(beleif.q.item(0))
     cth = np.cos(beleif.q.item(1))
     tth = np.tan(beleif.q.item(1))
     attitudeModelInversion = np.array([[1.0, sphi*tth, cphi*tth],
                                  [0.0, cphi, -sphi],
                                  [0.0, sphi/cth, cphi/cth]])
     print('Rb2v ekf = ', Rb2v.as_matrix())
     print('Rv2b ekf = ', Rv2b.as_matrix())
     ft.dp = Rb2v.apply(beleif.v.T).T
     print('dp = ', ft.dp)
     ft.dq = attitudeModelInversion @ omega
     print('dq = ', ft.dq)
     ft.dv = accel + Rv2b.apply(gravity.T).T# - np.cross(omega.T,beleif.v.T).T
     print('dv = ', ft.dv)
     ft.dba = np.array([[0.0,0.0,0.0]]).T
     ft.dbg = np.array([[0.0,0.0,0.0]]).T
     
     return ft

def update_gps_measurement_model(beleif):
     h = np.concatenate((beleif.p,beleif.v),axis=0)
     return h

def update_compass_measurement_model(beleif):
     h = np.array([[beleif.q.item(2)]]).T
     return h

def update_Jacobian_A(beleif,omega):
     #TODO: check Jacobian more thouroughly
     g = np.array([[0.0,0.0,9.81]]).T
     spsi = np.sin(beleif.q.item(2))
     cpsi = np.cos(beleif.q.item(2))

     dpdp = np.zeros((3,3))
     dpdq = np.array([[spsi*beleif.v.item(2), cpsi*beleif.v.item(2), spsi*beleif.v.item(0)-cpsi*beleif.v.item(1)],
                      [0.0, spsi*beleif.v.item(2), cpsi*beleif.v.item(0)-spsi*beleif.v.item(1)],
                      [beleif.v.item(1),-beleif.v.item(0),0.0]])
     dpdv = R.from_rotvec(beleif.q.squeeze()).as_matrix()
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

     At = np.concatenate((dpdx,dqdx,dvdx,dBadx,dBgdx),axis=0)
     return At

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
     

