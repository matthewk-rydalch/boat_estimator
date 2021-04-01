import numpy as np

class DynamicModel:
   def __init__(self,ft):
      self.dp = ft[0:3]
      self.dq = ft[3:6]
      self.dv = ft[6:9]
      self.dba = ft[9:12]
      self.dbg = ft[12:15]

      self.m = 15
      self.n = 1

   def __sub__(self,other):
      ddp = other.dp - self.dp
      ddq = other.dq - self.dq
      ddv = other.dv - self.dv
      ddba = other.dba - self.dba
      ddbg = other.ddbg - self.dbg
      ddf =  np.concatenate((ddp,ddq,ddv,ddba,ddbg),axis=0)

      return ddf