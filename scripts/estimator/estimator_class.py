import navpy
import numpy as np
import math

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
      self.stop = False

   def imu_callback(self,imu):
      if self.firstImu:
         self.imuPrevTime = imu.time
         self.firstImu = False
         return
      dt = imu.time - self.imuPrevTime
      self.imuPrevTime = imu.time

      ut = Inputs(comp_filter.run(self.belief,imu,dt,self.params.kp))
      ft = DynamicModel(ekf.update_dynamic_model(self.belief,ut))
      At = ekf.update_jacobian_A(self.belief,ut)
      Bt = ekf.update_jacobian_B(self.belief,ut)

      ekf.propagate(self.belief,self.params.RProcess,self.params.RInputs,ft,At,Bt,dt)
      self.update_full_state(ut.phi,ut.theta)

   def relPos_callback(self,relPos):
      if relPos.flags[-3] != '1':
         print('relPos not valid = ', relPos.flags)
         return
      zt = relPos.base2RoverRelPos
      ht = ekf.update_rtk_relPos_model(self.belief.p)
      Ct = ekf.get_jacobian_C_relPos()

      ekf.update(self.belief,self.params.QtRtk,zt,ht,Ct)

   def rover_gps_callback(self,gps):
      if gps.fix != 3:
         print('rover gps not in fix.  Fix = ', gps.fix)
         return
      if not self.refLlaSet:
         self.latRef = gps.lla[0]
         self.lonRef = gps.lla[1]
         self.altRef = gps.lla[2]
         refEcef1D = navpy.lla2ecef(self.latRef,self.lonRef,self.altRef)
         self.refEcef = np.array([refEcef1D]).T
         self.refLlaSet = True
         print('ref lla = ', self.latRef, ' ', self.lonRef, ' ', self.altRef)

      velocityNed1D = navpy.ecef2ned(gps.velocityEcef, self.latRef, self.lonRef, self.altRef)
      velocityNed = np.array([velocityNed1D]).T

      zt = velocityNed
      ht = ekf.update_rover_gps_velocity_model(self.belief.vr)
      Ct = ekf.get_jacobian_C_rover_velocity()

      ekf.update(self.belief,self.params.QtGpsVelocity,zt,ht,Ct)

   def base_gps_callback(self,gps):
      if self.stop == True:
         return
      if gps.fix != 3:
         print('base gps not in fix.  Fix = ', gps.fix)
         return
      if not self.refLlaSet:
         return
      positionEcefLocal = gps.positionEcef - self.refEcef
      positionNed1D = navpy.ecef2ned(positionEcefLocal,self.latRef,self.lonRef,self.altRef)
      positionNed = np.array([positionNed1D]).T
      velocityNed1D = navpy.ecef2ned(gps.velocityEcef, self.latRef, self.lonRef, self.altRef)
      velocityNed = np.array([velocityNed1D]).T

      zt = np.concatenate((positionNed,velocityNed),axis=0)
      ht = ekf.update_base_gps_velocity_model(self.baseStates.euler,self.belief.vb)
      Ct = ekf.get_jacobian_C_base_velocity(self.baseStates)

      ekf.update(self.belief,self.params.QtGps,zt,ht,Ct)

   def gps_compass_callback(self,gpsCompass):
      if gpsCompass.flags[2] != '1':
         print('Compass not valid = ', gpsCompass.flags)
         return
      zt = gpsCompass.heading*np.pi/180.0
      ht = ekf.update_rtk_compass_model(self.belief.psi)
      Ct = ekf.get_jacobian_C_compass()
      
      ekf.update(self.belief,self.params.QtRtkCompass,zt,ht,Ct)

   def update_full_state(self,phi,theta):
      self.baseStates.p = self.belief.P
      self.baseStates.euler = np.array([[phi,theta,self.belief.psi]])
      self.baseStates.vb = self.belief.vb