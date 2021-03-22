import numpy as np
from scipy.spatial.transform import Rotation as R

from states import States

def propagate():
    p = [0.0,0.0,0.0]
    q = [0.0,0.0,0.0,1.0]
    v = [0.0,0.0,0.0]
    ba = [0.0,0.0,0.0]
    bg = [0.0,0.0,0.0]
    cov = [1.0,1.0,1.0]
    xHat = States(p,q,v,ba,bg,cov,cov,cov,cov,cov)
    P = [[1.0,0.0,0.0],
         [0.0,1.0,0.0],
         [0.0,0.0,1.0]]
    return xHat, P

def dynamics(xHat,accel,omega,dt):
    accel = accel - xHat.ba
    omega = omega - xHat.bg
    Ra = R.from_quat(xHat.q)
    Rp = R.from_quat(xHat.q) ###TODO: one of these needs to be done different
    xHat.dp = Ra.apply(xHat.v)
    xHat.dth = omega
    xHat.dv = accel + Rp.apply([0.0,0.0,9.81]) - np.cross(omega,xHat.v)
    xHat.ba = np.array([0.0,0.0,0.0])
    xHat.bg = np.array([0.0,0.0,0.0])

    xHat.p = xHat.p + xHat.dp*dt
    th = R.from_rotvec(Ra.as_rotvec() + xHat.dth*dt) #TODO: this is wrong!  Need to do actual quaternion addition or other rotation addition
    xHat.q = th.as_quat()
    xHat.v = xHat.v + xHat.dv*dt
    xHat.ba = xHat.ba + xHat.dba*dt
    xHat.bg = xHat.bg + xHat.dbg*dt
    
    G = [[0.0,0.0,0.0],
         [0.0,0.0,0.0],
         [0.0,0.0,0.0]]

    return xHat, G