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