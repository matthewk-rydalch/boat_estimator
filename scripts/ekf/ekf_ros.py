#!/usr/bin/env python3

import rospy
from scipy.spatial.transform import Rotation as R

import sys
sys.path.append('/home/matt/px4_ws/src/boat_estimator/scripts/structs')
sys.path.append('/home/matt/px4_ws/src/boat_estimator/params')

from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from ublox.msg import PosVelEcef
from ublox.msg import RelPos

from sensors import ImuMsg
from sensors import GpsMsg
from sensors import GpsCompassMsg

from ekf_params import EKFParams
from ekf_class import EKF

class EKFRos:
    def __init__(self):
        self.odomEstimate = Odometry()
        params = EKFParams()
        self.ekf = EKF(params)

        self.boat_estimate_pub_ = rospy.Publisher('boat_odom', Odometry, queue_size=5, latch=True)
        self.imu_sub_ = rospy.Subscriber('imu', Imu, self.imuCallback, queue_size=5)
        self.pos_vel_ecef_sub_ = rospy.Subscriber('posVelEcef', PosVelEcef, self.posVelEcefCallback, queue_size=5)
        self.comp_rel_pos_sub_ = rospy.Subscriber('compass_relPos', RelPos, self.compassRelPosCallback, queue_size=5)
        self.ref_lla_sub_ = rospy.Subscriber('ref_lla',Vector3, self.refLlaCallback,queue_size=5)

        while not rospy.is_shutdown():
            rospy.spin()

    def imuCallback(self,msg):
        timeSeconds = msg.header.stamp.secs + msg.header.stamp.nsecs*1E-9
        gyrosDegreesPerSecond = [msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z] #TODO:check!  This is probably radians
        accelerometersMetersPerSecondSquared = [msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z]
        imu = ImuMsg(timeSeconds,accelerometersMetersPerSecondSquared,gyrosDegreesPerSecond)

        self.ekf.imu_callback(imu)
        self.publish_odom_estimate()

    def posVelEcefCallback(self,msg):
        positionEcefMeters = msg.position
        velocityEcefMetersPerSecond = msg.velocity
        gps = GpsMsg(positionEcefMeters,velocityEcefMetersPerSecond)
 
        self.ekf.gps_callback(gps)

    def compassRelPosCallback(self,msg):
        #TODO Name this something else?  Need to check and see if what we are receiving really is heading or if it is yaw
        headingDeg = msg.relPosHeading
        gpsCompass = GpsCompassMsg(headingDeg)

        self.ekf.gps_compass_callback(gpsCompass)

    def refLlaCallback(self,msg):
        self.ekf.set_ref_lla_callback(msg.x,msg.y,msg.z)

    def publish_odom_estimate(self):
        self.odomEstimate.header.stamp = rospy.Time.now()

        self.odomEstimate.pose.pose.position.x = self.ekf.beleif.p[0]
        self.odomEstimate.pose.pose.position.y = self.ekf.beleif.p[1]
        self.odomEstimate.pose.pose.position.z = self.ekf.beleif.p[2]

        quat = R.from_euler('xyz', self.ekf.beleif.q.T, degrees=False).as_quat()
        self.odomEstimate.pose.pose.orientation.x = quat.item(0)
        self.odomEstimate.pose.pose.orientation.y = quat.item(1)
        self.odomEstimate.pose.pose.orientation.z = quat.item(2)
        self.odomEstimate.pose.pose.orientation.w = quat.item(3)

        #These are in the body frame I beleive
        self.odomEstimate.twist.twist.linear.x = self.ekf.beleif.v[0]
        self.odomEstimate.twist.twist.linear.y = self.ekf.beleif.v[1]
        self.odomEstimate.twist.twist.linear.z = self.ekf.beleif.v[2]

        self.boat_estimate_pub_.publish(self.odomEstimate)


if __name__ == '__main__':
    rospy.init_node('ekf_ros', anonymous=True)
    try:
        ekfRos = EKFRos()
    except:
        rospy.ROSInterruptException
    pass

