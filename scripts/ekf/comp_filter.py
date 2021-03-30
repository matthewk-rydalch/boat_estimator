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
     Rb2v2 = R.from_euler('xyz',[0.0,0.0,beleif.q[0]])
     gravity = np.array([[0.0,0.0,9.81]]).T
     aBody = -imu.accelerometers + beleif.ba
     qAccel = np.array([[0.0,0.0,0.0]]).T
     qAccel[0] = np.arctan2(aBody.item(1),aBody.item(2))
     aV2 = Rb2v2.apply(aBody.T).T
     qAccel[1] = np.arcsin(aV2.item(0)/gravity.item(2))
     qError = qAccel - beleif.q
     print('qError = ', qError)
     qError[2][0] = 0.0
     dBg = attitudeModelInversion.T@(-ki*qError*dt)
     beleif.bg = beleif.bg + dBg
     omega = imu.gyros - beleif.bg
     dq = attitudeModelInversion @ omega + kp*qError
     beleif.q = beleif.q + dq*dt
