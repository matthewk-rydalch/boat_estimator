import navpy
import numpy as np

import sys
sys.path.append('../params')

from ekf_params import EKFParams
import ekf
from states import States

class EKF:
   def __init__(self):
      self.params = EKFParams()
      self.xHat = States(self.params.p0,self.params.q0,self.params.v0,self.params.ba0,self.params.bg0, \
         self.params.pCov0,self.params.qCov0,self.params.vCov0,self.params.baCov0,self.params.bgCov0)
      self.latRef = 0.0
      self.lonRef = 0.0
      self.altRef = 0.0
      self.imuPrevTime = 0.0

   def imu_callback(self,imu):
   #    covariance?
   #    covariance?
      if self.imuPrevTime == 0.0:
         return
      dt = imu.timeS - self.imuPrevTime
      u = [imu.accelerometers,imu.gyros]
      self.run_prediction_step(u,dt)

   def gps_callback(self,gps):
      #Start with simple ecef and velocity.  Add covariance and flags stuff later.
      # horizontal_covariance?
      # vertical_covariance?
      # speed_covariance?

      refEcef = np.array([navpy.lla2ecef(self.latRef,self.lonRef,self.altRef)]).T
      positionNed = np.array([navpy.ecef2ned((gps.positionEcef-refEcef),self.latRef,self.lonRef,self.altRef)]).T
      velocityNed = np.array([navpy.ecef2ned(gps.velocityEcef, self.latRef, self.lonRef, self.altRef)]).T
      zt = np.concatenate((positionNed,velocityNed),axis=0)
      ht = ekf.update_gps_measurement_model(self.xHat)
      C = ekf.get_jacobian_C_gps()
      self.run_correction_step(self.params.QtGps,zt,ht,C)

   def gps_compass_callback(self,gpsCompass):
   #    covariance?
   #    flag?
   #TODO: need to take into account the current orientation of the vehicle
      zt = gpsCompass.heading
      ht = ekf.update_compass_measurement_model(self.xHat)
      C = ekf.get_jacobian_C_compass()
      self.run_correction_step(self.params.QtGpsCompass,zt,ht,C)

   def run_prediction_step(self,u,dt):
      ekf.update_dynamic_model(self.xHat,u,dt)
      ekf.update_Jacobian_A(self.xHat,u[1])
      ekf.propagate(self.xHat,self.params.Rt,dt)

   def run_correction_step(self,Qt,zt,ht,C):
      ekf.update(self.xHat,Qt,zt,ht,C)