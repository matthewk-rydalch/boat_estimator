import numpy as np

def run(beleif,imu,dt,alpha):
     accel = imu.accelerometers - beleif.ba
     omega = imu.gyros - beleif.bg

     sphi = np.sin(beleif.q.item(0))
     cphi = np.cos(beleif.q.item(0))
     cth = np.cos(beleif.q.item(1))
     tth = np.tan(beleif.q.item(1))
     attitudeModelInversion = np.array([[1.0, sphi * tth, cphi * tth],
                                        [0.0, cphi, -sphi],
                                        [0.0, sphi / cth, cphi / cth]])
     dq = attitudeModelInversion @ omega
     gyroRoll = beleif.q[0] + dq[0]*dt
     gyroPitch = beleif.q[1] + dq[1]*dt
     accelRoll = np.arctan2(accel[0],accel[2])
     accelPitch = np.arctan2(accel[1],accel[2])
     beleif.q[0] = gyroRoll*alpha + accelRoll*(1-alpha)
     beleif.q[1] = gyroPitch*alpha + accelPitch*(1 - alpha)