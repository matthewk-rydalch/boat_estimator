import numpy as np

class DynamicModel:
   def __init__(self,ft):
      self.dp = ft[0:3]
      self.dvr = ft[3:6]
      self.dpsi = ft[6]
      self.dvb = ft[7:10]