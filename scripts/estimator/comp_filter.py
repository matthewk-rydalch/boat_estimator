import numpy as np

def run(belief,ut,dt,kp,ki,gravity):
     #I added the attitude model inversion.  It seemed to help.  It is not from Dr. Beard
     sphi = np.sin(belief.q.item(0))
     cphi = np.cos(belief.q.item(0))
     cth = np.cos(belief.q.item(1))
     tth = np.tan(belief.q.item(1))
     attitudeModelInversion = np.array([[1.0, sphi*tth, cphi*tth],
                                  [0.0, cphi, -sphi],
                                  [0.0, sphi/cth, cphi/cth]])
                                  
     aBody = -ut.accelerometers + belief.ba
     qAccel = np.array([[0.0,0.0,0.0]]).T
     qAccel[0] = np.arctan2(aBody.item(1),aBody.item(2))
     qAccel[1] = np.arcsin(aBody.item(0)/-gravity.item(2))
     qAccel[2] = belief.q[2] #We update this with rtk compassing
     qError = qAccel - belief.q

     dBg = attitudeModelInversion.T@(-ki*qError*dt)
     belief.bg = belief.bg + dBg
     omega = ut.gyros - belief.bg

     dq = attitudeModelInversion @ omega + kp*qError
     belief.q = belief.q + dq*dt
