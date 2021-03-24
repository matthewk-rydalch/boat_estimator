from sensors import ImuMsg
from sensors import GpsMsg
from sensors import GpsCompassMsg

from ekf_class import EKF

class EKFRos:
    def __init__(self):
        self.ekf = EKF()

    def imuCallback(self,msg):
        timeS = msg.header.stamp.secs + msg.header.stamp.nsecs*1E-9
        gyros = msg.angular_velocity
        accelerometers = msg.linear_acceleration
        imu = ImuMsg(timeS,accelerometers,gyros)

        self.ekf.imu_callback(imu)

    def posVelEcefCallback(self,msg):
        positionEcefMeters = msg.position
        velocityEcefMpS = msg.velocity #Mps: m/s
        gps = GpsMsg(positionEcefMeters,velocityEcefMpS)
 
        self.ekf.gps_callback(gps)

    def compassRelPosCallback(self,msg):
        headingDeg = msg.relPosHeading
        gpsCompass = GpsCompassMsg(headingDeg)

        self.ekf.gps_compass_callback(gpsCompass)
