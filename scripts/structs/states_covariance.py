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

      self.m = 15
      self.n = 1

   def get_copy(self):
      xCopy = StatesCovariance(np.copy(self.p),np.copy(self.q),np.copy(self.v),np.copy(self.ba),np.copy(self.bg),np.copy(self.P))
      return xCopy

   def add_to_item(self,index,addedValue):
      switcher = {
      0: self.setPn,
      1: self.setPe,
      2: self.setPd,
      3: self.setPhi,
      4: self.setTheta,
      5: self.setPsi,
      6: self.setU,
      7: self.setV,
      8: self.setW,
      9: self.setBax,
      10: self.setBay,
      11: self.setBaz,
      12: self.setBgx,
      13: self.setBgy,
      14: self.setBgz
      }
      func = switcher.get(index, "invalid index")
      func(addedValue)
   def setPn(self,addedValue):
      self.p[0][0] += addedValue
   def setPe(self,addedValue):
      self.p[1][0] += addedValue
   def setPd(self,addedValue):
      self.p[2][0] += addedValue
   def setPhi(self,addedValue):
      self.q[0][0] += addedValue
   def setTheta(self,addedValue):
      self.q[1][0] += addedValue
   def setPsi(self,addedValue):
      self.q[2][0] += addedValue
   def setU(self,addedValue):
      self.v[0][0] += addedValue
   def setV(self,addedValue):
      self.v[1][0] += addedValue
   def setW(self,addedValue):
      self.v[2][0] += addedValue
   def setBax(self,addedValue):
      self.ba[0][0] += addedValue
   def setBay(self,addedValue):
      self.ba[1][0] += addedValue
   def setBaz(self,addedValue):
      self.ba[2][0] += addedValue
   def setBgx(self,addedValue):
      self.bg[0][0] += addedValue
   def setBgy(self,addedValue):
      self.bg[1][0] += addedValue
   def setBgz(self,addedValue):
      self.bg[2][0] += addedValue
