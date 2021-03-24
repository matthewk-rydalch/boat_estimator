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

   def run_prediction_step(self,u,dt):
      ekf.update_dynamic_model(self.xHat,u,dt)
      ekf.update_Jacobian_A(self.xHat,u[1])
      ekf.propagate(self.xHat,self.params.Rt,dt)

   def run_correction_step(self,zt):
      ht = ekf.update_measurement_model(self.xHat)
      C = ekf.get_jacobian_C()
      ekf.update(self.xHat,self.params.Qt,zt,ht,C)