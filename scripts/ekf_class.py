import sys
sys.path.append('../params')

from ekf_params import EKFParams
import ekf
from states import States

class EKF:
   def __init__(self):
      self.xHat = States(EKFParams.p0,EKFParams.q0,EKFParams.v0,EKFParams.ba0,EKFParams.bg0, \
         EKFParams.pCov0,EKFParams.qCov0,EKFParams.vCov0,EKFParams.baCov0,EKFParams.bgCov0)

   def prediction_step(self,u,dt):
      ekf.dynamics(self.xHat,u,dt)
      ekf.update_Jacobian_A(self.xHat,u[1])
      ekf.propagate(self.xHat,EKFParams.Rt,dt)

   def correction_step(self):
      a = 1