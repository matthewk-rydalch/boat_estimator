from sensors import ImuMsg
from sensors import GpsMsg
from sensors import GpsCompassMsg

from ekf_class import EKF

class EKFRos:
    def __init__(self):
        self.ekf = EKF()

    def imuCallback(self,msg):
        imu = ImuMsg()
        imu.angularVelocity = msg.angular_velocity
        imu.linearAcceleration = msg.linear_acceleration

        self.ekf.imu_callback(imu)

    def posVelEcefCallback(self,msg):
        gps = GpsMsg()
        gps.ecef = msg.position
        gps.velocity = msg.velocity

        self.ekf.gps_callback(gps)

    def compassRelPosCallback(self,msg):
        gpsCompass = GpsCompassMsg()
        gpsCompass.heading = msg.relPosHeading

        self.ekf.gps_compass_callback()
