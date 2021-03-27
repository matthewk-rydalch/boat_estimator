#!/usr/bin/env python3

import numpy as np
import navpy
from scipy.spatial.transform import Rotation as R

def compute_truth(t,truth):
    truth.position[0] = np.sin(t) + 3.0
    truth.position[1] = -0.5*np.sin(t) + 1.0
    truth.position[2] = 0.8*np.cos(t) - 3.0
    truth.orientation[0] = 0.02*np.sin(t)
    truth.orientation[1] = 0.03*np.sin(t)
    truth.orientation[2] = -0.01*np.sin(t)
    Rb2i = R.from_euler('xyz',np.squeeze(truth.orientation))
    
    xDot = np.cos(t)
    yDot = -0.5*np.cos(t)
    zDot = -0.8*np.sin(t)
    bodyVelocity = Rb2i.apply([xDot,yDot,zDot])
    truth.velocity[0] = bodyVelocity[0]
    truth.velocity[1] = bodyVelocity[1]
    truth.velocity[2] = bodyVelocity[2]
    
    sth = np.sin(truth.orientation[1])
    cth = np.cos(truth.orientation[1])
    cphi = np.cos(truth.orientation[0])
    sphi = np.sin(truth.orientation[0])
    phiDot = 0.02*np.cos(t)
    thetaDot = 0.03*np.cos(t)
    psiDot = -0.01*np.cos(t)
    truth.angularVelocity[0] = phiDot - sth*psiDot #These come from euler dynamics
    truth.angularVelocity[1] = cphi*thetaDot + sphi*cth*psiDot
    truth.angularVelocity[2] = -sphi*thetaDot + cphi*cth*psiDot

    truth.acceleration[0] = -np.sin(t)
    truth.acceleration[1] = 0.5*np.sin(t)
    truth.acceleration[2] = -0.8*np.cos(t)

def compute_imu(truth,imu):
    Rb2i = R.from_euler('xyz',np.squeeze(truth.orientation))
    Ri2b = Rb2i.inv()

    imu.gyros = truth.angularVelocity
    velocityBody = Ri2b.apply(np.squeeze(truth.velocity))

    corriolisEffect = np.cross(np.squeeze(truth.angularVelocity),velocityBody)
    gravity = np.array([[0.0,0.0,9.81]]).T
    feltAccelerationInertial = truth.acceleration - gravity #+ np.array([corriolisEffect]).T
    accelBody = Ri2b.apply(np.squeeze(feltAccelerationInertial))
    imu.accelerometers[0] = accelBody[0]
    imu.accelerometers[1] = accelBody[1]
    imu.accelerometers[2] = accelBody[2]

def compute_gps(truth,gps,latRef,lonRef,altRef,originEcef):
    ecefPositionRelative = navpy.ned2ecef(truth.position,latRef,lonRef,altRef)
    gps.position = ecefPositionRelative + originEcef
    gps.velocity = navpy.ned2ecef(truth.velocity,latRef,lonRef,altRef)

def compute_gps_compass(truth,gpsCompass):
    #TODO should the compass direction take into account the roll and pitch?
    gpsCompass.heading = truth.orientation[2]