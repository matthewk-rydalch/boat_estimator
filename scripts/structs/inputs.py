import numpy as np

class Inputs:
   def __init__(self,ut):
      self.accelerometers = ut[0:3]
      self.gyros = ut[3:6]
      self.phi = ut[6]
      self.theta = ut[7]