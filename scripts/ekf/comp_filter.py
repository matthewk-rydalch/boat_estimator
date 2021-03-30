import numpy as np

def run(beleif,imu,dt,kp,ki):
     gravity = np.array([[0.0,0.0,9.81]]).T
     qAccel = np.array([[0.0,0.0,0.0]]).T
     qAccel[0] = np.arctan2(imu.accelerometers.item(1)-beleif.ba.item(1),imu.accelerometers.item(2)-beleif.ba.item(2))
     qAccel[1] = np.arcsin((imu.accelerometers.item(0)-beleif.ba.item(0))/gravity.item(2))
     qError = qAccel - beleif.q
     qError[2][0] = 0.0
     dBg = -ki*qError*dt
     beleif.bg = beleif.bg + dBg
     dq = imu.gyros - beleif.bg + kp*qError

     beleif.q = beleif.q + dq*dt
     #TODO: Do I need to rotate into the right frame?
