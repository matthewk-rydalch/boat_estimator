import navpy
import numpy as np

import sys
sys.path.append('/home/matt/px4_ws/src/boat_estimator/src/structs')

import ekf
import comp_filter
from states_covariance import StatesCovariance
from dynamic_model import DynamicModel

class Estimator:
   def __init__(self,params):
      self.params = params
      self.belief = StatesCovariance(self.params.pr0,self.params.vr0,self.params.p0,self.params.q0, \
         self.params.v0,self.params.ba0,self.params.bg0, self.params.P0)
      self.refLlaSet = False
      self.latRef = 0.0
      self.lonRef = 0.0
      self.altRef = 0.0
      self.refEcef = np.zeros((3,1))
      self.imuPrevTime = 0.0
      self.firstImu = True

   def imu_callback(self,imu):
      if self.firstImu:
         self.imuPrevTime = imu.time
         self.firstImu = False
         return
      dt = imu.time - self.imuPrevTime
      self.imuPrevTime = imu.time

      ft = DynamicModel(ekf.update_dynamic_model(self.belief,imu))
      At = ekf.get_numerical_jacobian(ekf.update_dynamic_model,self.belief,imu)
      Bt = ekf.update_jacobian_B(self.belief)

      comp_filter.run(self.belief,imu,dt,self.params.kp,self.params.ki,self.params.gravity)
      ekf.propagate(self.belief,self.params.RProcess,self.params.RImu,ft,At,Bt,dt)

   def relPos_callback(self,relPos):
      #TODO: Add flag checks
      zt = relPos.base2RoverRelPos
      ht = ekf.update_relPos_measurement_model(self.belief)
      Ct = ekf.get_jacobian_C_relPos()

      ekf.update(self.belief,self.params.QtRtk,zt,ht,Ct)

   def rover_gps_callback(self,gps):
      #TODO: Add flag checks
      if not self.refLlaSet:
         return

      velocityNed1D = navpy.ecef2ned(gps.velocityEcef, self.latRef, self.lonRef, self.altRef)
      velocityNed = np.array([velocityNed1D]).T

      zt = velocityNed
      ht = ekf.update_rover_velocity_measurement_model(self.belief)
      Ct = ekf.get_jacobian_C_rover_velocity()

      ekf.update(self.belief,self.params.QtGpsVelocity,zt,ht,Ct)

   def base_gps_callback(self,gps):
      #TODO: Add flag checks
      if not self.refLlaSet:
         return

      positionEcefLocal = gps.positionEcef - self.refEcef
      positionNed1D = navpy.ecef2ned(positionEcefLocal,self.latRef,self.lonRef,self.altRef)
      positionNed = np.array([positionNed1D]).T
      velocityNed1D = navpy.ecef2ned(gps.velocityEcef, self.latRef, self.lonRef, self.altRef)
      velocityNed = np.array([velocityNed1D]).T

      zt = np.concatenate((positionNed,velocityNed),axis=0)
      ht = ekf.update_base_gps_measurement_model(self.belief,gps)
      Ct = ekf.get_numerical_jacobian(ekf.update_base_gps_measurement_model,self.belief,gps)

      ekf.update(self.belief,self.params.QtGps,zt,ht,Ct)

   def gps_compass_callback(self,gpsCompass):
      #TODO: Add flag check
      #TODO: Do I need to take into account the current orientation of the vehicle?
      zt = gpsCompass.heading*np.pi/180.0
      ht = ekf.update_compass_measurement_model(self.belief)
      Ct = ekf.get_jacobian_C_compass()
      
      ekf.update(self.belief,self.params.QtRtkCompass,zt,ht,Ct)

   def set_ref_lla_callback(self,latDegrees,lonDegrees,altMeters):
      self.latRef = latDegrees
      self.lonRef = lonDegrees
      self.altRef = altMeters
      refEcef1D = navpy.lla2ecef(self.latRef,self.lonRef,self.altRef)
      self.refEcef = np.array([refEcef1D]).T
      self.refLlaSet = True