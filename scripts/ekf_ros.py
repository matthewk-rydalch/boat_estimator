import rospy

from std_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from ublox.msg import PosVelEcef
from ublox.msg import RelPos

from sensors import ImuMsg
from sensors import GpsMsg
from sensors import GpsCompassMsg

from ekf_class import EKF

class EKFRos:
    def __init__(self):
        self.ekf = EKF()

        # self.rover2Base_relPos_stripped_pub_ = rospy.Publisher('rover2Base_relPos_stripped', Point, queue_size=5, latch=True)
        self.imu_sub_ = rospy.Subscriber('imu', Imu, self.imuCallback, queue_size=5)
        self.pos_vel_ecef_sub_ = rospy.Subscriber('posVelEcef', PosVelEcef, self.posVelEcefCallback, queue_size=5)
        self.comp_rel_pos_sub_ = rospy.Subscriber('compass_relPos', RelPos, self.compassRelPosCallback, queue_size=5)
        self.ref_lla_sub_ = rospy.Subscriber('ref_lla',Vector3, self.refLlaCallback,queue_size=5)
    
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

    def refLlaCallback(self,msg):
        self.ekf.set_ref_lla_callback(msg.x,msg.y,msg.z)
