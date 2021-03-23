import numpy as np
from scipy.spatial.transform import Rotation as R

from states import States

def propagate():
    p = [0.0,0.0,0.0]
    th = [0.0,0.0,0.0]
    v = [0.0,0.0,0.0]
    ba = [0.0,0.0,0.0]
    bg = [0.0,0.0,0.0]
    cov = [1.0,1.0,1.0]
    xHat = States(p,th,v,ba,bg,cov,cov,cov,cov,cov)
    P = [[1.0,0.0,0.0],
         [0.0,1.0,0.0],
         [0.0,0.0,1.0]]
    return xHat, P

def dynamics(xHat,accel,omega,dt):
     accel = accel - xHat.ba
     omega = omega - xHat.bg
     Rb2v = R.from_rotvec(xHat.th)
     Rv2b = Rb2v.inv()
     sphi = np.sin(xHat.th[0])
     cphi = np.cos(xHat.th[0])
     cth = np.cos(xHat.th[1])
     tth = np.tan(xHat.th[1])
     attitudeDynamics = np.array([[1.0, sphi*tth, cphi*tth],
                                  [0.0, cphi, -sphi],
                                  [0.0, sphi/cth, cphi/cth]])

     xHat.dp = Rb2v.apply(xHat.v)
     xHat.dth = attitudeDynamics @ omega
     xHat.dv = accel + Rv2b.apply([0.0,0.0,9.81]) - np.cross(omega,xHat.v)
     xHat.ba = np.array([0.0,0.0,0.0])
     xHat.bg = np.array([0.0,0.0,0.0])

     xHat.p = xHat.p + xHat.dp*dt
     xHat.th = xHat.th + xHat.dth*dt
     xHat.v = xHat.v + xHat.dv*dt
     xHat.ba = xHat.ba + xHat.dba*dt
     xHat.bg = xHat.bg + xHat.dbg*dt

     G = [[0.0,0.0,0.0],
          [0.0,0.0,0.0],
          [0.0,0.0,0.0]]

     return xHat, G