import numpy as np

def run(beleif,imu,dt,alpha):
     gyroRoll = beleif.q[0] + imu.gyros[0]*dt
     gyroPitch = beleif.q[1] + imu.gyros[1] * dt
     accelRoll = np.arctan(imu.accelerometers[0]/imu.accelerometers[2])
     accelPitch = np.arctan(imu.accelerometers[1]/imu.accelerometers[2])
     beleif.q[0] = gyroRoll*alpha + accelRoll*(1-alpha)
     beleif.q[1] = gyroPitch*alpha + accelPitch*(1 - alpha)