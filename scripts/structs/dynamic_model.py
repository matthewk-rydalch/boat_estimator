import numpy as np

class DynamicModel:
   def __init__(self,ft):
      self.dpr = ft[0:3]
      self.dvr = ft[3:6]
      self.dp = ft[6:9]
      self.dq = ft[9:12]
      self.dv = ft[12:15]
      self.dba = ft[15:18]
      self.dbg = ft[18:21]

      self.m = 21
      self.n = 1