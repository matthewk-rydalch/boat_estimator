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

from sensors import TruthMsg,ImuMsg,GpsMsg,RelPosMsg,GpsCompassMsg
import synthetic_measurements

class SyntheticMeasurements:
    def __init__(self):
        self.truthRos = Odometry()
        self.roverTruthRos = Odometry()
        self.imuRos = Imu()
        self.gpsRos = PosVelEcef()
        self.roverGpsRos = PosVelEcef()
        self.relPosRos = RelPos()
        self.gpsCompassRos = RelPos()

        self.truth = TruthMsg()
        self.roverTruth = TruthMsg()
        self.imu = ImuMsg()
        self.gps = GpsMsg()
        self.roverGps = GpsMsg()
        self.base2RoverRelPos = RelPosMsg()
        self.gpsCompass = GpsCompassMsg

        self.accelerometerAccuracyStdDev = 0.025
        self.gyroAccuracyStdDev = 0.0023
        self.gpsHorizontalAccuracyStdDev = 0.2
        self.gpsVerticalAccuracyStdDev = 0.4
        self.gpsSpeedAccuracyStdDev = 0.2
        self.rtkHorizontalAccuracyStdDev = 0.02
        self.rtkVerticalAccuracyStdDev = 0.04
        self.gpsCompassAccuracyDegStdDev = 1.0 #depends on baseline. This also uses RTK

        #These are to remember noise of previous update.  They are used in the low pass filter
        self.gpsNoise = [0.0,0.0,0.0,0.0,0.0,0.0]
        self.gpsCompassNoise = 0.0
        
        self.accelerometerBias = [0.3,-0.5,-0.2]
        self.gyroBias = [0.01,0.08,-0.02]

        self.imuTs = 1.0/200.0
        self.gpsTs = 1.0/5.0

        self.firstCallback = True
        self.firstTime = 0.0

        self.latRef = 20.24
        self.lonRef = -111.66
        self.altRef = 1387.0
        self.originEcef = navpy.lla2ecef(self.latRef,self.lonRef,self.altRef)

        self.gravity = np.array([[0.0,0.0,9.81]]).T
        self.lowPassFilterAlpha = 0.9

        self.truth_pub_ = rospy.Publisher('truth',Odometry,queue_size=5,latch=True)
        self.rover_truth_pub_ = rospy.Publisher('rover_truth',Odometry,queue_size=5,latch=True)
        self.imu_pub_ = rospy.Publisher('imu',Imu,queue_size=5,latch=True)
        self.gps_pub_ = rospy.Publisher('gps',PosVelEcef,queue_size=5,latch=True)
        self.rover_gps_pub_ = rospy.Publisher('rover_gps',PosVelEcef,queue_size=5,latch=True)
        self.relPos_pub_ = rospy.Publisher('base2Rover_relPos',RelPos,queue_size=5,latch=True)
        self.gps_compass_pub_ = rospy.Publisher('gps_compass',RelPos,queue_size=5,latch=True)

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

        synthetic_measurements.compute_rover_truth(timeSeconds,self.roverTruth)
        self.publish_rover_truth(stamp,self.roverTruth)

        synthetic_measurements.compute_imu(self.truth,self.imu,self.gravity)
        synthetic_measurements.add_imu_noise(self.imu,self.accelerometerAccuracyStdDev,self.gyroAccuracyStdDev)
        synthetic_measurements.add_imu_bias(self.imu,self.accelerometerBias,self.gyroBias)
        self.publish_imu(stamp,self.imu)      

    def gpsCallback(self,event):
        if self.firstCallback:
            return
        stamp = rospy.Time.now()
        
        synthetic_measurements.compute_gps(self.truth,self.gps,self.latRef,self.lonRef,self.altRef,self.originEcef)
        synthetic_measurements.add_gps_noise(self.gps,self.gpsHorizontalAccuracyStdDev,self.gpsVerticalAccuracyStdDev, \
            self.gpsSpeedAccuracyStdDev,self.latRef,self.lonRef,self.altRef,self.lowPassFilterAlpha,self.gpsNoise)
        # TODO: Add gps random walk
        self.publish_gps(stamp,self.gps)

        synthetic_measurements.compute_rover_gps(self.roverTruth,self.roverGps,self.latRef,self.lonRef,self.altRef,self.originEcef)
        synthetic_measurements.add_gps_noise(self.roverGps,self.gpsHorizontalAccuracyStdDev,self.gpsVerticalAccuracyStdDev, \
            self.gpsSpeedAccuracyStdDev,self.latRef,self.lonRef,self.altRef,self.lowPassFilterAlpha,self.gpsNoise)
        # TODO: Add gps random walk
        self.publish_rover_gps(stamp,self.roverGps)

        synthetic_measurements.compute_rover_relative_position(self.truth,self.roverTruth,self.base2RoverRelPos)
        synthetic_measurements.add_rtk_noise(self.base2RoverRelPos,self.rtkHorizontalAccuracyStdDev,self.rtkVerticalAccuracyStdDev)
        self.publish_relPos(stamp,self.base2RoverRelPos)

        synthetic_measurements.compute_gps_compass(self.truth,self.gpsCompass)
        synthetic_measurements.add_gps_compass_noise(self.gpsCompass,self.gpsCompassAccuracyDegStdDev,self.lowPassFilterAlpha, \
            self.gpsCompassNoise)
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

    def publish_rover_truth(self,stamp,truth):
        self.roverTruthRos.header.stamp = stamp
        self.roverTruthRos.pose.pose.position.x = truth.position[0]
        self.roverTruthRos.pose.pose.position.y = truth.position[1]
        self.roverTruthRos.pose.pose.position.z = truth.position[2]
        quat = R.from_euler('xyz',np.squeeze(truth.orientation)).as_quat()
        self.roverTruthRos.pose.pose.orientation.x = quat[0]
        self.roverTruthRos.pose.pose.orientation.y = quat[1]
        self.roverTruthRos.pose.pose.orientation.z = quat[2]
        self.roverTruthRos.pose.pose.orientation.w = quat[3]
        self.roverTruthRos.twist.twist.linear.x = truth.velocity[0]
        self.roverTruthRos.twist.twist.linear.y = truth.velocity[1]
        self.roverTruthRos.twist.twist.linear.z = truth.velocity[2]
        self.roverTruthRos.twist.twist.angular.x = truth.angularVelocity[0]
        self.roverTruthRos.twist.twist.angular.y = truth.angularVelocity[1]
        self.roverTruthRos.twist.twist.angular.z = truth.angularVelocity[2]

        self.rover_truth_pub_.publish(self.roverTruthRos)

    def publish_imu(self,stamp,imu):
        self.imuRos.header.stamp = stamp
        self.imuRos.linear_acceleration.x = imu.accelerometers[0]
        self.imuRos.linear_acceleration.y = imu.accelerometers[1]
        self.imuRos.linear_acceleration.z = imu.accelerometers[2]
        self.imuRos.angular_velocity.x = imu.gyros[0]
        self.imuRos.angular_velocity.y = imu.gyros[1]
        self.imuRos.angular_velocity.z = imu.gyros[2]

        self.imu_pub_.publish(self.imuRos)

    def publish_gps(self,stamp,gps):
        self.gpsRos.header.stamp = stamp
        self.gpsRos.position = gps.positionEcef
        self.gpsRos.velocity = gps.velocityEcef
        self.gpsRos.fix = gps.fix

        self.gps_pub_.publish(self.gpsRos)

    def publish_rover_gps(self,stamp,gps):
        self.roverGpsRos.header.stamp = stamp
        self.roverGpsRos.position = gps.positionEcef
        self.roverGpsRos.velocity = gps.velocityEcef
        self.roverGpsRos.lla = gps.lla
        self.roverGpsRos.fix = gps.fix

        self.rover_gps_pub_.publish(self.roverGpsRos)

    def publish_relPos(self,stamp,relPos):
        self.relPosRos.relPosNED[0] = relPos.base2RoverRelPos.item(0)
        self.relPosRos.relPosNED[1] = relPos.base2RoverRelPos.item(1)
        self.relPosRos.relPosNED[2] = relPos.base2RoverRelPos.item(2)
        self.relPosRos.flags = 311

        self.relPos_pub_.publish(self.relPosRos)

    def publish_gps_compass(self,stamp,gpsCompass):
        self.gpsCompassRos.header.stamp = stamp
        self.gpsCompassRos.relPosHeading = gpsCompass.heading
        self.gpsCompassRos.flags = gpsCompass.flags

        self.gps_compass_pub_.publish(self.gpsCompassRos)

if __name__ == '__main__':
    rospy.init_node('synthetic_measurements', anonymous=True)
    try:
        syncMeas = SyntheticMeasurements()
    except:
        rospy.ROSInterruptException
    pass