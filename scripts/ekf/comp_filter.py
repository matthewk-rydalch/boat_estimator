import numpy as np
from scipy.spatial.transform import Rotation as R

def run(beleif,imu,dt,kp,ki):
     #TODO: I added the attitude model inversion.  It seemed to help.  It is not from Dr. Beard
     sphi = np.sin(beleif.q.item(0))
     cphi = np.cos(beleif.q.item(0))
     cth = np.cos(beleif.q.item(1))
     tth = np.tan(beleif.q.item(1))
     attitudeModelInversion = np.array([[1.0, sphi*tth, cphi*tth],
                                  [0.0, cphi, -sphi],
                                  [0.0, sphi/cth, cphi/cth]])
     gravity = np.array([[0.0,0.0,9.81]]).T
     qAccel = np.array([[0.0,0.0,0.0]]).T
     aBody = -imu.accelerometers + beleif.ba
     qAccel[0] = np.arctan2(aBody.item(1),aBody.item(2))
     qAccel[1] = np.arcsin(aBody.item(0)/-gravity.item(2))
     qError = qAccel - beleif.q
     qError[2][0] = 0.0
     dBg = attitudeModelInversion.T@(-ki*qError*dt)
     beleif.bg = beleif.bg + dBg
     omega = imu.gyros - beleif.bg
     dq = attitudeModelInversion @ omega + kp*qError
     # dq = kp*qError
     beleif.q = beleif.q + dq*dt
