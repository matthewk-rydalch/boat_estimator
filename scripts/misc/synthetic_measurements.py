#!/usr/bin/env python3

import numpy as np
import navpy
from scipy.spatial.transform import Rotation as R
import math

def compute_truth(t,truth,roverTruth):
    baseX = 2.0*t + 0.5*np.cos(t/2.0) - 3.0
    baseXDot = 2.0 + -0.5*np.sin(t/2.0)/2.0
    baseXDDot = -0.5*np.cos(t/2.0)/4.0
    roverX = 2.0*t + 0.5*np.cos(t/2.0) -3.0 + math.exp(2.0/(t+1.0))
    roverXDot = 2.0 + -0.5*np.sin(t/2.0)/2.0 + math.exp(2.0/(t+1.0))*(-2.0/((t+1.0)**2))

    baseY = -2.0*t -0.5*np.sin(t/2.0) + 5.0
    baseYDot = -2.0 + -0.5*np.cos(t/2.0)/2.0
    baseYDDot = 0.5*np.sin(t/2.0)/4.0
    roverY = -2.0*t -0.5*np.sin(t/2.0) + 5.0 + math.exp(2.0/(t+1.0))
    roverYDot = -2.0 + -0.5*np.cos(t/2.0)/2.0 + math.exp(2.0/(t+1.0))*(-2.0/((t+1.0)**2))

    baseZ = 0.5*np.sin(t/2.0) - 1.0
    baseZDot = 0.5*np.cos(t/2.0)/2.0
    baseZDDot = -0.5*np.sin(t/2.0)/4.0
    roverZ = 0.5*np.sin(t/2.0) - 1.0 + math.exp(2.0/(t+1.0))
    roverZDot = 0.5*np.cos(t/2.0)/2.0 + math.exp(2.0/(t+1.0))*(-2.0/((t+1.0)**2))

    basePhi = 0.05*np.sin(t/2.0)
    basePhiDot = 0.05*np.cos(t/2.0)/2.0

    baseTheta = 0.05*np.sin(t/2.0)
    baseThetaDot = 0.05*np.cos(t/2.0)/2.0

    basePsi = -0.1*np.sin(t/2.0)
    basePsiDot = -0.1*np.cos(t/2.0)/2.0

    truth.position[0] = baseX - roverX
    truth.position[1] = baseY - roverY
    truth.position[2] = baseZ - roverZ

    Rb2i = R.from_euler('xyz',np.squeeze(truth.orientation))
    Ri2b = Rb2i.inv()
    baseBodyVelocity = Ri2b.apply([baseXDot,baseYDot,baseZDot])
    truth.velocity[0] = baseBodyVelocity[0]
    truth.velocity[1] = baseBodyVelocity[1]
    truth.velocity[2] = baseBodyVelocity[2]

    baseBodyAcceleration = Ri2b.apply([baseXDDot,baseYDDot,baseZDDot])
    truth.acceleration[0] = baseBodyAcceleration[0]
    truth.acceleration[1] = baseBodyAcceleration[1]
    truth.acceleration[2] = baseBodyAcceleration[2]
    
    truth.orientation[0] = basePhi
    truth.orientation[1] = baseTheta
    truth.orientation[2] = basePsi

    cphi = np.cos(basePhi)
    sphi = np.sin(basePhi)
    sth = np.sin(baseTheta)
    cth = np.cos(baseTheta)
    truth.angularVelocity[0] = basePhiDot - sth*basePsiDot #These come from euler dynamics
    truth.angularVelocity[1] = cphi*baseThetaDot + sphi*cth*basePsiDot
    truth.angularVelocity[2] = -sphi*baseThetaDot + cphi*cth*basePsiDot

    roverTruth.position = np.array([[roverX,roverY,roverZ]]).T
    roverTruth.velocity = np.array([[roverXDot,roverYDot,roverZDot]]).T

def compute_imu(truth,imu,gravity):
    Rb2i = R.from_euler('xyz',np.squeeze(truth.orientation))
    Ri2b = Rb2i.inv()

    imu.gyros = truth.angularVelocity

    feltAcceleration = truth.acceleration - Ri2b.apply(gravity.T).T #should not include corriolis effect!
    imu.accelerometers[0] = feltAcceleration[0]
    imu.accelerometers[1] = feltAcceleration[1]
    imu.accelerometers[2] = feltAcceleration[2]

def compute_rover_relPos(truth,base2RoverRelPos):
    base2RoverRelPos.base2RoverRelPos = -truth.position

    base2RoverRelPos.flags = 311

def compute_base_gps(truth,gps,latRef,lonRef,altRef,originEcef):   
    Rb2i = R.from_euler('xyz',truth.orientation.squeeze())
    trueVelocityNed = Rb2i.apply(truth.velocity.T).T
    gps.velocityEcef = navpy.ned2ecef(trueVelocityNed,latRef,lonRef,altRef)

    gps.fix = 3

def compute_rover_gps(roverTruth,gps,latRef,lonRef,altRef,originEcef):
    gps.lla = navpy.ned2lla(roverTruth.position,latRef,lonRef,altRef)

    ecefPositionRelative = navpy.ned2ecef(roverTruth.position,latRef,lonRef,altRef)
    gps.positionEcef = ecefPositionRelative + originEcef
    
    gps.velocityEcef = navpy.ned2ecef(roverTruth.velocity,latRef,lonRef,altRef)

    gps.fix = 3

def compute_rtk_compass(truth,rtkCompass):
    rtkCompass.heading = truth.orientation[2]*180.0/np.pi

    rtkCompass.flags = 311

def add_imu_noise(imu,accelerometerAccuracy,gyroAccuracy):
    imu.accelerometers[0] = np.random.normal(imu.accelerometers[0],accelerometerAccuracy)
    imu.accelerometers[1] = np.random.normal(imu.accelerometers[1],accelerometerAccuracy)
    imu.accelerometers[2] = np.random.normal(imu.accelerometers[2],accelerometerAccuracy)
    imu.gyros[0] = np.random.normal(imu.gyros[0],gyroAccuracy)
    imu.gyros[1] = np.random.normal(imu.gyros[1],gyroAccuracy)
    imu.gyros[2] = np.random.normal(imu.gyros[2],gyroAccuracy)

def add_gps_noise(gps,gpsHorizontalAccuracy,gpsVerticalAccuracy,gpsSpeedAccuracy,latRef,lonRef,altRef,alpha,gpsNoise):
    gpsEcefAccuracy = navpy.ned2ecef([gpsHorizontalAccuracy,gpsHorizontalAccuracy,gpsVerticalAccuracy],latRef,lonRef,altRef)
    whiteNoisePositionEcef = [0]*3
    whiteNoiseVelocityEcef = [0]*3
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

def add_rtk_noise(relPos,rtkHorizontalAccuracyStdDev,rtkVerticalAccuracyStdDev):
    for i in range(2):
        relPos.base2RoverRelPos[i] = np.random.normal(relPos.base2RoverRelPos[i],rtkHorizontalAccuracyStdDev)
    relPos.base2RoverRelPos[2] = np.random.normal(relPos.base2RoverRelPos[2],rtkVerticalAccuracyStdDev)

def add_rtk_compass_noise(rtkCompass,rtkCompassAccuracy,alpha,rtkCompassNoise):
    whiteNoise = np.random.normal(0.0,rtkCompassAccuracy)
    rtkCompassNoise = low_pass_filter(rtkCompassNoise,whiteNoise,alpha)
    rtkCompass.heading = rtkCompass.heading + rtkCompassNoise

def add_imu_bias(imu,accelerometerBias,gyroBias):
    imu.accelerometers = imu.accelerometers + np.array([accelerometerBias]).T
    imu.gyros = imu.gyros + np.array([gyroBias]).T

def low_pass_filter(y,u,alpha):
    yNew = alpha*y+(1-alpha)*u
    return yNew