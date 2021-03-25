#!/usr/bin/env python3

import rospy
import numpy as np
import navpy
from scipy.spatial.transform import Rotation as R

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from ublox.msg import PosVelEcef
from ublox.msg import RelPos

class SyntheticMeasurements:
    def __init__(self):
        self.truth = Odometry()
        self.imu = Imu()
        self.gps = PosVelEcef()
        self.gpsCompass = RelPos()
        self.refLla = PosVelEcef()
        self.acceleration = np.zeros((3,1))

        #TODO: Add variation to the periods?
        self.imuTs = 1.0/200
        self.gpsTs = 1.0/5.0

        self.firstCallback = True
        self.firstTime = 0.0
        self.refLlaSent = False

        self.latRef = 20.24
        self.lonRef = -111.66
        self.altRef = 1387.0
        self.originEcef = navpy.lla2ecef(self.latRef,self.lonRef,self.altRef)

        self.truth_pub_ = rospy.Publisher('truth',Odometry,queue_size=5,latch=True)
        self.imu_pub_ = rospy.Publisher('imu',Imu,queue_size=5,latch=True)
        self.gps_pub_ = rospy.Publisher('gps',PosVelEcef,queue_size=5,latch=True)
        self.gps_compass_pub_ = rospy.Publisher('gps_compass',RelPos,queue_size=5,latch=True)
        self.ref_lla_pub_ = rospy.Publisher('ref_lla',PosVelEcef,queue_size=5,latch=True)

        self.truth_rate_timer_ = rospy.Timer(rospy.Duration(self.imuTs), self.truthCallback)
        self.gps_rate_timer_ = rospy.Timer(rospy.Duration(self.gpsTs), self.gpsCallback)

        while not rospy.is_shutdown():
            rospy.spin()

    def truthCallback(self,event):
        stamp = rospy.Time.now()
        if self.firstCallback:
            self.firstTime = stamp.secs+stamp.nsecs*1e-9
            self.firstCallback = False
        timeSeconds = stamp.secs+stamp.nsecs*1e-9 - self.firstTime

        self.compute_truth(timeSeconds,stamp)
        self.truth_pub_.publish(self.truth)

        self.compute_imu()
        # self.add_imu_noise()
        # self.add_imu_bias()
        self.imu_pub_.publish(self.imu)

    def gpsCallback(self,event):
        if self.firstCallback:
            return
        stamp = rospy.Time.now()
        if not self.refLlaSent:
            self.setup_ref_lla(stamp)
            self.ref_lla_pub_.publish(self.refLla)
            self.refLlaSent = True
        
        self.compute_gps(stamp)
        # self.add_gps_noise()
        self.gps_pub_.publish(self.gps)

        self.compute_gps_compass(stamp)
        # self.add_gps_compass_noise()
        self.gps_compass_pub_.publish(self.gpsCompass)

    def compute_truth(self,t,stamp):
        self.truth.header.stamp = stamp
        self.truth.pose.pose.position.x = np.sin(t)
        self.truth.pose.pose.position.y = t
        self.truth.pose.pose.position.z = np.cos(t)
        self.truth.pose.pose.orientation.x = 0.0
        self.truth.pose.pose.orientation.y = 0.0
        self.truth.pose.pose.orientation.z = 0.0
        self.truth.pose.pose.orientation.w = 1.0
        self.truth.twist.twist.linear.x = np.cos(t)
        self.truth.twist.twist.linear.y = 1.0
        self.truth.twist.twist.linear.z = -np.sin(t)
        self.truth.twist.twist.angular.x = 0.0
        self.truth.twist.twist.angular.y = 0.0
        self.truth.twist.twist.angular.z = 0.0

        self.acceleration[0] = -np.sin(t)
        self.acceleration[1] = 0.0
        self.acceleration[2] = -np.cos(t)

    def compute_imu(self):
        self.imu.header.stamp = self.truth.header.stamp
        self.imu.angular_velocity = self.truth.twist.twist.angular
        #TODO: add gravity and coriolis?
        self.imu.linear_acceleration.x = self.acceleration[0]
        self.imu.linear_acceleration.y = self.acceleration[1]
        self.imu.linear_acceleration.z = self.acceleration[2]

    def setup_ref_lla(self,stamp):
        self.refLla.header.stamp = stamp
        self.refLla.lla = [self.latRef,self.lonRef,self.altRef]

    def compute_gps(self,stamp):
        self.gps.header.stamp = stamp
        truePositionNed = [self.truth.pose.pose.position.x,self.truth.pose.pose.position.y,self.truth.pose.pose.position.z]
        ecefPositionRelative = navpy.ned2ecef(truePositionNed,self.latRef,self.lonRef,self.altRef)
        self.gps.position = ecefPositionRelative + self.originEcef
        trueVelocityNed = [self.truth.twist.twist.linear.x,self.truth.twist.twist.linear.y,self.truth.twist.twist.linear.z]
        self.gps.velocity = navpy.ned2ecef(trueVelocityNed,self.latRef,self.lonRef,self.altRef)

    def compute_gps_compass(self,stamp):
        #TODO should the compass direction take into account the roll and pitch?
        self.gpsCompass.header.stamp = stamp
        quat = [self.truth.pose.pose.orientation.x,self.truth.pose.pose.orientation.y,self.truth.pose.pose.orientation.z,self.truth.pose.pose.orientation.w]
        rotation = R.from_quat(quat)
        eulerRadians = rotation.as_euler('zyx')
        self.gpsCompass.relPosHeading = eulerRadians[2]

if __name__ == '__main__':
    rospy.init_node('synthetic_measurements', anonymous=True)
    try:
        syncMeas = SyntheticMeasurements()
    except:
        rospy.ROSInterruptException
    pass