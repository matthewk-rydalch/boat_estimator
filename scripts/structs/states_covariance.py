import numpy as np

class StatesCovariance:
   def __init__(self,pMeters,qDegrees,vMetersPerSecond,baMetersPerSecondSquared,bgDegreesPerSecond,PCovariance):
      #TODO need to make units clear and type (numpy array?)
      self.p = pMeters
      self.q = qDegrees
      self.v = vMetersPerSecond
      self.ba = baMetersPerSecondSquared
      self.bg = bgDegreesPerSecond

      self.P = PCovariance
