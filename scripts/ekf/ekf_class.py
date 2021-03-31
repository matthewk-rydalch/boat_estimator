import navpy
import numpy as np

import sys
sys.path.append('/home/matt/px4_ws/src/boat_estimator/params')
sys.path.append('/home/matt/px4_ws/src/boat_estimator/src/structs')

from ekf_params import EKFParams
import ekf
import comp_filter
from states_covariance import StatesCovariance
from dynamic_model import DynamicModel

class EKF:
   def __init__(self):
      self.params = EKFParams()
      self.beleif = StatesCovariance(self.params.p0,self.params.q0,self.params.v0,self.params.ba0,self.params.bg0, \
         self.params.P0)
      self.refLlaSet = False
      self.latRef = 0.0
      self.lonRef = 0.0
      self.altRef = 0.0
      self.imuPrevTime = 0.0
      self.firstImu = True

   def imu_callback(self,imu):
      #TODO: Add covariance values
      if self.firstImu:
         self.imuPrevTime = imu.time
         self.firstImu = False
         return
      dt = imu.time - self.imuPrevTime
      self.imuPrevTime = imu.time
      ut = [imu.accelerometers,imu.gyros]
      ft = DynamicModel()
      ekf.update_dynamic_model(ft,self.beleif,ut,self.params.gravity,dt)
      At = ekf.update_jacobian_A(self.beleif,imu.gyros)
      Bt = ekf.update_jacobian_B(self.beleif)
      comp_filter.run(self.beleif,imu,dt,self.params.kp,self.params.ki)
      ekf.propagate(self.beleif,self.params.RProcess,self.params.RImu,ft,At,Bt,dt)

   def gps_callback(self,gps):
      #TODO: Add covariance values
      #TODO: Add flag checks
      if not self.refLlaSet:
         return
      refEcef = navpy.lla2ecef(self.latRef,self.lonRef,self.altRef)
      refEcef = np.array([refEcef]).T
      positionEcefLocal = gps.positionEcef - refEcef
      positionNed1D = navpy.ecef2ned(positionEcefLocal,self.latRef,self.lonRef,self.altRef)
      positionNed = np.array([positionNed1D]).T
      velocityNed1D = navpy.ecef2ned(gps.velocityEcef, self.latRef, self.lonRef, self.altRef)
      velocityNed = np.array([velocityNed1D]).T
      zt = np.concatenate((positionNed,velocityNed),axis=0)
      ht = ekf.update_gps_measurement_model(self.beleif)
      #TODO: Should probably set up the C jacobians in the parameter file
      Ct = ekf.get_jacobian_C_gps()
      ekf.update(self.beleif,self.params.QtGps,zt,ht,Ct)

   def gps_compass_callback(self,gpsCompass):
      #TODO: Add covariance values
      #TODO: Add flag check
      #TODO: Do I need to take into account the current orientation of the vehicle?
      zt = gpsCompass.heading
      ht = ekf.update_compass_measurement_model(self.beleif)
      #TODO: Should probably set up the C jacobians in the parameter file
      Ct = ekf.get_jacobian_C_compass()
      ekf.update(self.beleif,self.params.QtGpsCompass,zt,ht,Ct)

   def set_ref_lla_callback(self,latDegrees,lonDegrees,altMeters):
      self.latRef = latDegrees
      self.lonRef = lonDegrees
      self.altRef = altMeters
      self.refLlaSet = True