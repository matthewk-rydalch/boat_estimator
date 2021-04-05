#!/usr/bin/env python3

import numpy as np
import rospy
from scipy.spatial.transform import Rotation as R

import sys
sys.path.append('/home/matt/px4_ws/src/boat_estimator/scripts/structs')
sys.path.append('/home/matt/px4_ws/src/boat_estimator/params')

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from ublox.msg import PosVelEcef
from ublox.msg import RelPos

from sensors import ImuMsg
from sensors import RelPosMsg
from sensors import GpsMsg
from sensors import GpsCompassMsg

from estimator_params_class import EstimatorParams
from estimator_class import Estimator

class EstimatorRos:
    def __init__(self):
        self.relPosEstimate = Vector3Stamped()
        self.odomEstimate = Odometry()
        params = EstimatorParams()
        self.estimator = Estimator(params)

        self.boat_estimate_pub_ = rospy.Publisher('boat_odom', Odometry, queue_size=5, latch=True)
        self.boat_relative_position_estimate_pub_ = rospy.Publisher('boat_relPos', Vector3Stamped, queue_size=5, latch=True)
        self.imu_sub_ = rospy.Subscriber('imu', Imu, self.imuCallback, queue_size=5)
        self.base_2_rover_relPos_sub_ = rospy.Subscriber('base_2_rover_relPos', RelPos, self.relPosCallback, queue_size=5)
        self.rover_pos_vel_ecef_sub_ = rospy.Subscriber('rover_posVelEcef', PosVelEcef, self.roverPosVelEcefCallback, queue_size=5)
        self.base_pos_vel_ecef_sub_ = rospy.Subscriber('base_posVelEcef', PosVelEcef, self.basePosVelEcefCallback, queue_size=5)
        self.comp_relPos_sub_ = rospy.Subscriber('compass_relPos', RelPos, self.compassRelPosCallback, queue_size=5)

        while not rospy.is_shutdown():
            rospy.spin()

    def imuCallback(self,msg):
        timeSeconds = msg.header.stamp.secs + msg.header.stamp.nsecs*1E-9
        gyrosRadiansPerSecond = [msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z]
        accelerometersMetersPerSecondSquared = [msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z]
        imu = ImuMsg(timeSeconds,accelerometersMetersPerSecondSquared,gyrosRadiansPerSecond)

        self.estimator.imu_callback(imu)
        self.publish_estimates()

    def relPosCallback(self,msg):
        base2RoverRelativePositionNedMeters = np.array(msg.relPosNED) + np.array(msg.relPosHPNED)
        flags = bin(msg.flags)
        relPos = RelPosMsg(base2RoverRelativePositionNedMeters,flags)

        self.estimator.relPos_callback(relPos)

    def roverPosVelEcefCallback(self,msg):
        positionEcefMeters = msg.position
        velocityEcefMetersPerSecond = msg.velocity
        latLonAltDegM = msg.lla
        fix = msg.fix
        gps = GpsMsg(positionEcefMeters,velocityEcefMetersPerSecond,latLonAltDegM,fix)
 
        self.estimator.rover_gps_callback(gps)

    def basePosVelEcefCallback(self,msg):
        positionEcefMeters = msg.position
        velocityEcefMetersPerSecond = msg.velocity
        latLonAltDegM = msg.lla
        fix = msg.fix
        gps = GpsMsg(positionEcefMeters,velocityEcefMetersPerSecond,latLonAltDegM,fix)
 
        self.estimator.base_gps_callback(gps)

    def compassRelPosCallback(self,msg):
        #TODO Need to check and see if what we are receiving really is heading or if it is the z rotation in a frame that is rolled and pitched.  It is probably heading.
        headingDeg = msg.relPosHeading
        flags = bin(msg.flags)
        gpsCompass = GpsCompassMsg(headingDeg,flags)

        self.estimator.gps_compass_callback(gpsCompass)

    def publish_estimates(self):
        timeStamp = rospy.Time.now()
        
        self.relPosEstimate.header.stamp = timeStamp

        self.relPosEstimate.vector.x = self.estimator.belief.pr[0]
        self.relPosEstimate.vector.y = self.estimator.belief.pr[1]
        self.relPosEstimate.vector.z = self.estimator.belief.pr[2]

        self.boat_relative_position_estimate_pub_.publish(self.relPosEstimate)

        self.odomEstimate.header.stamp = timeStamp

        self.odomEstimate.pose.pose.position.x = self.estimator.belief.p[0]
        self.odomEstimate.pose.pose.position.y = self.estimator.belief.p[1]
        self.odomEstimate.pose.pose.position.z = self.estimator.belief.p[2]

        quat = R.from_euler('xyz', self.estimator.belief.q.T, degrees=False).as_quat()
        self.odomEstimate.pose.pose.orientation.x = quat.item(0)
        self.odomEstimate.pose.pose.orientation.y = quat.item(1)
        self.odomEstimate.pose.pose.orientation.z = quat.item(2)
        self.odomEstimate.pose.pose.orientation.w = quat.item(3)

        self.odomEstimate.twist.twist.linear.x = self.estimator.belief.v[0]
        self.odomEstimate.twist.twist.linear.y = self.estimator.belief.v[1]
        self.odomEstimate.twist.twist.linear.z = self.estimator.belief.v[2]

        self.boat_estimate_pub_.publish(self.odomEstimate)

if __name__ == '__main__':
    rospy.init_node('estimator_ros', anonymous=True)
    try:
        estimatorRos = EstimatorRos()
    except:
        rospy.ROSInterruptException
    pass

