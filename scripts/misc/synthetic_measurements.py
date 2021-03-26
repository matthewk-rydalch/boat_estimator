#!/usr/bin/env python3

import rospy
import numpy as np
import navpy
from scipy.spatial.transform import Rotation as R

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from ublox.msg import PosVelEcef
from ublox.msg import RelPos
from geometry_msgs.msg import Vector3

class SyntheticMeasurements:
    def __init__(self):
        self.truth = Odometry()
        self.imu = Imu()
        self.gps = PosVelEcef()
        self.gpsCompass = RelPos()
        self.refLla = Vector3()
        self.acceleration = np.zeros((3,1))
        self.RTruth = R.from_euler('xyz',[0.0,0.0,0.0])

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
        self.ref_lla_pub_ = rospy.Publisher('ref_lla',Vector3,queue_size=5,latch=True)

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
            self.refLla.x = self.latRef
            self.refLla.y = self.lonRef
            self.refLla.z = self.altRef
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
        self.truth.pose.pose.position.x = 0.0#3.0*np.cos(t) - 3.0
        self.truth.pose.pose.position.y = 0.0#np.sin(t)
        self.truth.pose.pose.position.z = 0.0#np.cos(t)
        truePhi = 0.2*np.sin(t)
        trueTheta = 0.0#0.3*np.sin(t)
        truePsi = 0.0#-0.1*np.sin(t)
        self.RTruth = R.from_euler('xyz',[truePhi,trueTheta,truePsi])
        trueQuaternion = self.RTruth.as_quat()
        self.truth.pose.pose.orientation.x = trueQuaternion[0]
        self.truth.pose.pose.orientation.y = trueQuaternion[1]
        self.truth.pose.pose.orientation.z = trueQuaternion[2]
        self.truth.pose.pose.orientation.w = trueQuaternion[3]
        
        sth = np.sin(trueTheta)
        cphi = np.cos(truePhi)
        sphi = np.sin(truePhi)
        cth = np.cos(trueTheta)
        phiDot = 0.2*np.cos(t)
        thetaDot = 0.0 #0.3*np.cos(t)
        psiDot = 0.0 #-0.1*np.cos(t)
        self.truth.twist.twist.linear.x = 0.0#2.0*t#-3.0*np.sin(t)
        self.truth.twist.twist.linear.y = 0.0#np.cos(t)
        self.truth.twist.twist.linear.z = 0.0#-np.sin(t)
        self.truth.twist.twist.angular.x = phiDot - sth*psiDot #These come from euler dynamics
        self.truth.twist.twist.angular.y = cphi*thetaDot + sphi*cth*psiDot
        self.truth.twist.twist.angular.z = -sphi*thetaDot + cphi*cth*psiDot

        self.acceleration[0] = 0.0#-3.0*np.cos(t)
        self.acceleration[1] = 0.0#-np.sin(t)
        self.acceleration[2] = 0.0#-np.cos(t)

    def compute_imu(self):
        self.imu.header.stamp = self.truth.header.stamp
        Ri2b = self.RTruth.inv()
        angularVelocityInertial = [self.truth.twist.twist.angular.x,self.truth.twist.twist.angular.y,self.truth.twist.twist.angular.z]
        angularVelocityBody = Ri2b.apply(angularVelocityInertial)
        self.imu.angular_velocity.x = angularVelocityBody[0]
        self.imu.angular_velocity.y = angularVelocityBody[1]
        self.imu.angular_velocity.z = angularVelocityBody[2]
        velocityInertial = [self.truth.twist.twist.linear.x,self.truth.twist.twist.linear.y,self.truth.twist.twist.linear.z]
        velocityBody = Ri2b.apply(velocityInertial)
        corriolisEffect = np.cross(angularVelocityBody,velocityBody)
        feltAccelerationInertial = self.acceleration - np.array([[0.0,0.0,9.81]]).T #+ np.array([corriolisEffect]).T
        accelBody = Ri2b.apply(np.squeeze(feltAccelerationInertial))
        self.imu.linear_acceleration.x = accelBody[0]
        self.imu.linear_acceleration.y = accelBody[1]
        self.imu.linear_acceleration.z = accelBody[2]

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
        eulerRadians = rotation.as_euler('xyz')
        self.gpsCompass.relPosHeading = eulerRadians[2]

if __name__ == '__main__':
    rospy.init_node('synthetic_measurements', anonymous=True)
    try:
        syncMeas = SyntheticMeasurements()
    except:
        rospy.ROSInterruptException
    pass