import numpy as np

class DynamicModel:
   def __init__(self):
      self.dp = np.zeros((3,1))
      self.dq = np.zeros((3,1))
      self.dv = np.zeros((3,1))
      self.dba = np.zeros((3,1))
      self.dbg = np.zeros((3,1))