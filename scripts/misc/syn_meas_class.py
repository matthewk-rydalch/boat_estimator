#!/usr/bin/env python3

import rospy
import numpy as np
import navpy
from scipy.spatial.transform import Rotation as R

import sys
sys.path.append('/home/matt/px4_ws/src/boat_estimator/scripts/structs')

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from ublox.msg import PosVelEcef
from ublox.msg import RelPos
from geometry_msgs.msg import Vector3

from sensors import TruthMsg,ImuMsg,GpsMsg,GpsCompassMsg
import synthetic_measurements

class SyntheticMeasurements:
    def __init__(self):
        self.truthRos = Odometry()
        self.imuRos = Imu()
        self.gpsRos = PosVelEcef()
        self.gpsCompassRos = RelPos()
        self.refLla = Vector3()

        self.truth = TruthMsg()
        self.imu = ImuMsg()
        self.gps = GpsMsg()
        self.gpsCompass = GpsCompassMsg

        self.accelerometerAccuracyStdDev = 0.025
        self.gyroAccuracyStdDev = 0.0023
        self.gpsHorizontalAccuracyStdDev = 0.2
        self.gpsVerticalAccuracyStdDev = 0.4
        self.gpsSpeedAccuracyStdDev = 0.2
        self.gpsCompassAccuracyStdDev = 0.02 #depends on baseline. This also uses RTK

        self.gpsNoise = [0.0,0.0,0.0,0.0,0.0,0.0]
        self.gpsCompassNoise = 0.0
        
        self.accelerometerBias = [0.1,-0.05,-0.02]
        self.gyroBias = [0.0,0.0,0.0]
        # self.gyroBias = [0.01,0.08,-0.02]

        #TODO: Add variation to the periods?
        self.imuTs = 1.0/200.0
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
        timeSeconds = stamp.secs+stamp.nsecs*1E-9 - self.firstTime

        synthetic_measurements.compute_truth(timeSeconds,self.truth)
        self.publish_truth(stamp,self.truth)

        synthetic_measurements.compute_imu(self.truth,self.imu)
        # synthetic_measurements.add_imu_noise(self.imu,self.accelerometerAccuracyStdDev,self.gyroAccuracyStdDev)
        # synthetic_measurements.add_imu_bias(self.imu,self.accelerometerBias,self.gyroBias)
        self.publish_imu(stamp,self.imu)      

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
        
        synthetic_measurements.compute_gps(self.truth,self.gps,self.latRef,self.lonRef,self.altRef,self.originEcef)
        # synthetic_measurements.add_gps_noise(self.gps,self.gpsHorizontalAccuracyStdDev,self.gpsVerticalAccuracyStdDev,self.gpsSpeedAccuracyStdDev,self.latRef,self.lonRef,self.altRef,self.gpsNoise)
        self.publish_gps(stamp,self.gps)

        synthetic_measurements.compute_gps_compass(self.truth,self.gpsCompass)
        # synthetic_measurements.add_gps_compass_noise(self.gpsCompass,self.gpsCompassAccuracyStdDev,self.gpsCompassNoise)
        # synthetic_measurements.add_gps_random_walk() #TODO is this in the wrong spot?
        self.publish_gps_compass(stamp,self.gpsCompass)

    def publish_truth(self,stamp,truth):
        self.truthRos.header.stamp = stamp
        self.truthRos.pose.pose.position.x = truth.position[0]
        self.truthRos.pose.pose.position.y = truth.position[1]
        self.truthRos.pose.pose.position.z = truth.position[2]
        quat = R.from_euler('xyz',np.squeeze(truth.orientation)).as_quat()
        self.truthRos.pose.pose.orientation.x = quat[0]
        self.truthRos.pose.pose.orientation.y = quat[1]
        self.truthRos.pose.pose.orientation.z = quat[2]
        self.truthRos.pose.pose.orientation.w = quat[3]
        self.truthRos.twist.twist.linear.x = truth.velocity[0]
        self.truthRos.twist.twist.linear.y = truth.velocity[1]
        self.truthRos.twist.twist.linear.z = truth.velocity[2]
        self.truthRos.twist.twist.angular.x = truth.angularVelocity[0]
        self.truthRos.twist.twist.angular.y = truth.angularVelocity[1]
        self.truthRos.twist.twist.angular.z = truth.angularVelocity[2]

        self.truth_pub_.publish(self.truthRos)

    def publish_imu(self,stamp,imu):
        self.imuRos.header.stamp = stamp
        self.imuRos.angular_velocity.x = imu.gyros[0]
        self.imuRos.angular_velocity.y = imu.gyros[1]
        self.imuRos.angular_velocity.z = imu.gyros[2]
        self.imuRos.linear_acceleration.x = imu.accelerometers[0]
        self.imuRos.linear_acceleration.y = imu.accelerometers[1]
        self.imuRos.linear_acceleration.z = imu.accelerometers[2]

        self.imu_pub_.publish(self.imuRos)

    def publish_gps(self,stamp,gps):
        self.gpsRos.header.stamp = stamp
        self.gpsRos.position = gps.positionEcef
        self.gpsRos.velocity = gps.velocityEcef

        self.gps_pub_.publish(self.gpsRos)

    def publish_gps_compass(self,stamp,gpsCompass):
        self.gpsCompassRos.header.stamp = stamp
        self.gpsCompassRos.relPosHeading = gpsCompass.heading

        self.gps_compass_pub_.publish(self.gpsCompassRos)

if __name__ == '__main__':
    rospy.init_node('synthetic_measurements', anonymous=True)
    try:
        syncMeas = SyntheticMeasurements()
    except:
        rospy.ROSInterruptException
    pass