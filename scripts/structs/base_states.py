import numpy as np

class BaseStates:
   def __init__(self,p0,euler0,vb0):
      self.p = p0  
      self.euler = euler0
      self.vb = vb0