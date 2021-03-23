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

   def prediction_step(self,u,dt):
      ekf.dynamics(self.xHat,u,dt)
      ekf.update_Jacobian_A(self.xHat,u[1])
      ekf.propagate(self.xHat,self.params.Rt,dt)

   def correction_step(self):
      a = 1