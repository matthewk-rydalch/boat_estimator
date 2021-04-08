import numpy as np

def run(fullState,imu,dt,kp):
     #I added the attitude model inversion.  It seemed to help.  It is not from Dr. Beard
     sphi = np.sin(fullState.euler.item(0))
     cphi = np.cos(fullState.euler.item(0))
     cth = np.cos(fullState.euler.item(1))
     tth = np.tan(fullState.euler.item(1))
     attitudeModelInversion = np.array([[1.0, sphi*tth, cphi*tth],
                                  [0.0, cphi, -sphi],
                                  [0.0, sphi/cth, cphi/cth]])
                                  
     aBody = -imu.accelerometers #TODO: Check why this is negative
     eulerAccel = np.array([[0.0,0.0,0.0]]).T
     eulerAccel[0] = np.arctan2(aBody.item(1),aBody.item(2))
     eulerAccel[1] = np.arctan2(aBody.item(0),aBody.item(2))
     eulerAccel[2] = fullState.euler.item(2) #We update this with rtk compassing
     eulerError = eulerAccel - fullState.euler

     dEuler = attitudeModelInversion @ imu.gyros + kp*eulerError
     
     phi = fullState.euler.item(0) + dEuler[0]*dt
     theta = fullState.euler.item(1) + dEuler[1]*dt

     ut = [imu.accelerometers.item(0),imu.accelerometers.item(1),imu.accelerometers.item(2), \
               imu.gyros.item(0),imu.gyros.item(1),imu.gyros.item(2), \
               phi, theta]

     return ut