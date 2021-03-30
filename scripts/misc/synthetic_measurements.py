#!/usr/bin/env python3

import numpy as np
import navpy
from scipy.spatial.transform import Rotation as R

def compute_truth(t,truth):

    truth.position[0] = 2*t + 0.5*np.cos(t/2.0) - 3.0
    xDot = 2.0 + -0.5*np.sin(t/2.0)/2.0
    xDDot = -0.5*np.cos(t/2.0)/4.0

    truth.position[1] = 0.3*t -0.5*np.sin(t/2.0) + 5.0
    yDot = 0.3 + -0.5*np.cos(t/2.0)/2.0
    yDDot = 0.5*np.sin(t/2.0)/4.0

    truth.position[2] = 0.5*np.sin(t/2.0) - 1.0
    zDot = 0.5*np.cos(t/2.0)/2.0
    zDDot = -0.5*np.sin(t/2.0)/4.0

    # truth.orientation[0] = 0.2
    # phiDot = 0.0

    # truth.orientation[1] = 0.0
    # thetaDot = 0.0

    # truth.orientation[2] = 0.0
    # psiDot = 0.0

    truth.orientation[0] = 0.05*np.sin(t/2.0)
    phiDot = 0.05*np.cos(t/2.0)/2.0

    truth.orientation[1] = 0.05*np.sin(t/2.0)
    thetaDot = 0.05*np.cos(t/2.0)/2.0

    truth.orientation[2] = -0.05*np.sin(t/2.0)
    psiDot = -0.05*np.cos(t/2.0)/2.0

    Rb2i = R.from_euler('xyz',np.squeeze(truth.orientation))
    Ri2b = Rb2i.inv()

    bodyVelocity = Ri2b.apply([xDot,yDot,zDot])
    truth.velocity[0] = bodyVelocity[0]
    truth.velocity[1] = bodyVelocity[1]
    truth.velocity[2] = bodyVelocity[2]
    
    sth = np.sin(truth.orientation[1])
    cth = np.cos(truth.orientation[1])
    cphi = np.cos(truth.orientation[0])
    sphi = np.sin(truth.orientation[0])
    truth.angularVelocity[0] = phiDot - sth*psiDot #These come from euler dynamics
    truth.angularVelocity[1] = cphi*thetaDot + sphi*cth*psiDot
    truth.angularVelocity[2] = -sphi*thetaDot + cphi*cth*psiDot

    bodyAcceleration = Ri2b.apply([xDDot,yDDot,zDDot])
    truth.acceleration[0] = bodyAcceleration[0]
    truth.acceleration[1] = bodyAcceleration[1]
    truth.acceleration[2] = bodyAcceleration[2]

def compute_imu(truth,imu):
    Rb2i = R.from_euler('xyz',np.squeeze(truth.orientation))
    Ri2b = Rb2i.inv()

    imu.gyros = truth.angularVelocity

    gravity = np.array([[0.0,0.0,9.81]]).T
    feltAcceleration = truth.acceleration - Ri2b.apply(gravity.T).T #should not include corriolis effect!
    imu.accelerometers[0] = feltAcceleration[0]
    imu.accelerometers[1] = feltAcceleration[1]
    imu.accelerometers[2] = feltAcceleration[2]

def compute_gps(truth,gps,latRef,lonRef,altRef,originEcef):
    ecefPositionRelative = navpy.ned2ecef(truth.position,latRef,lonRef,altRef)
    gps.positionEcef = ecefPositionRelative + originEcef
    gps.velocityEcef = navpy.ned2ecef(truth.velocity,latRef,lonRef,altRef)

def compute_gps_compass(truth,gpsCompass):
    #TODO should the compass direction take into account the roll and pitch?
    gpsCompass.heading = truth.orientation[2]

def add_imu_noise(imu,accelerometerAccuracy,gyroAccuracy):
    imu.accelerometers[0] = np.random.normal(imu.accelerometers[0],accelerometerAccuracy)
    imu.accelerometers[1] = np.random.normal(imu.accelerometers[1],accelerometerAccuracy)
    imu.accelerometers[2] = np.random.normal(imu.accelerometers[2],accelerometerAccuracy)
    imu.gyros[0] = np.random.normal(imu.gyros[0],gyroAccuracy)
    imu.gyros[1] = np.random.normal(imu.gyros[1],gyroAccuracy)
    imu.gyros[2] = np.random.normal(imu.gyros[2],gyroAccuracy)

def add_gps_noise(gps,gpsHorizontalAccuracy,gpsVerticalAccuracy,gpsSpeedAccuracy,latRef,lonRef,altRef,gpsNoise):
    gpsEcefAccuracy = navpy.ned2ecef([gpsHorizontalAccuracy,gpsHorizontalAccuracy,gpsVerticalAccuracy],latRef,lonRef,altRef)
    whiteNoisePositionEcef = [0]*3
    whiteNoiseVelocityEcef = [0]*3
    alpha = 0.9
    for i in range(3):
        whiteNoisePositionEcef[i] = np.random.normal(0.0,gpsEcefAccuracy[i])
        whiteNoiseVelocityEcef[i] = np.random.normal(0.0,gpsSpeedAccuracy)
        gpsNoise[i] = low_pass_filter(gpsNoise[i],whiteNoisePositionEcef[i],alpha)
        gpsNoise[i+3] = low_pass_filter(gpsNoise[i+3],whiteNoiseVelocityEcef[i],alpha)

    gps.positionEcef[0] = gps.positionEcef[0] + gpsNoise[0]
    gps.positionEcef[1] = gps.positionEcef[1] + gpsNoise[1]
    gps.positionEcef[2] = gps.positionEcef[2] + gpsNoise[2]
    gps.velocityEcef[0] = gps.velocityEcef[0] + gpsNoise[3]
    gps.velocityEcef[1] = gps.velocityEcef[1] + gpsNoise[4]
    gps.velocityEcef[2] = gps.velocityEcef[2] + gpsNoise[5]

def add_gps_compass_noise(gpsCompass,gpsCompassAccuracy,gpsCompassNoise):
    alpha = 0.9
    whiteNoise = np.random.normal(0.0,gpsCompassAccuracy)
    gpsCompassNoise = low_pass_filter(gpsCompassNoise,whiteNoise,alpha)
    gpsCompass.heading = gpsCompass.heading + gpsCompassNoise

def add_imu_bias(imu,accelerometerBias,gyroBias):
    imu.accelerometers = imu.accelerometers + np.array([accelerometerBias]).T
    imu.gyros = imu.gyros + np.array([gyroBias]).T

def add_gps_random_walk():
    a = 1

def low_pass_filter(y,u,alpha):
    yNew = alpha*y+(1-alpha)*u
    return yNew