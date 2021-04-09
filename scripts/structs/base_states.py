import numpy as np

class BaseStates:
   def __init__(self,p0,euler0,vb0):
      self.p = p0  
      self.euler = euler0
      self.vb = vb0
      self.wLpf = np.zeros((3,1))

      self.alpha = 0.1

   def update_w_lpf(self,gyros):
      self.wLpf = alpha*gyros+(1-alpha)*self.wLpf