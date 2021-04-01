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

   # def __getitem__(self,key):
   #    switcher = {
   #       0: self.p[0][0],
   #       1: self.p[1][0],
   #       2: self.p[2][0],
   #       3: self.q[0][0],
   #       4: self.q[1][0],
   #       5: self.q[2][0],
   #       6: self.v[0][0],
   #       7: self.v[1][0],
   #       8: self.v[2][0],
   #       9: self.ba[0][0],
   #       10: self.ba[1][0],
   #       11: self.ba[2][0],
   #       12: self.bg[0][0],
   #       13: self.bg[1][0],
   #       14: self.bg[2][0]
   #    }
   #    return switcher.get(key,"invalid index")

   # def __setitem__(self,key,newValue):
   #    switcher = {
   #       0: self.setPn,
   #       1: self.setPe,
   #       2: self.setPd,
   #       3: self.setPhi,
   #       4: self.setTheta,
   #       5: self.setPsi,
   #       6: self.setU,
   #       7: self.setV,
   #       8: self.setW,
   #       9: self.setBax,
   #       10: self.setBay,
   #       11: self.setBaz,
   #       12: self.setBgx,
   #       13: self.setBgy,
   #       14: self.setBgz
   #    }
   #    func = switcher.get(key, "invalid index")
   #    func(newValue)
   # def setPn(self,newValue):
   #    self.p[0][0] = newValue
   # def setPe(self,newValue):
   #    self.p[1][0] = newValue
   # def setPd(self,newValue):
   #    self.p[2][0] = newValue
   # def setPhi(self,newValue):
   #    self.q[0][0] = newValue
   # def setTheta(self,newValue):
   #    self.q[1][0] = newValue
   # def setPsi(self,newValue):
   #    self.q[2][0] = newValue
   # def setU(self,newValue):
   #    self.v[0][0] = newValue
   # def setV(self,newValue):
   #    self.v[1][0] = newValue
   # def setW(self,newValue):
   #    self.v[2][0] = newValue
   # def setBax(self,newValue):
   #    self.ba[0][0] = newValue
   # def setBay(self,newValue):
   #    self.ba[1][0] = newValue
   # def setBaz(self,newValue):
   #    self.ba[2][0] = newValue
   # def setBgx(self,newValue):
   #    self.bg[0][0] = newValue
   # def setBgy(self,newValue):
   #    self.bg[1][0] = newValue
   # def setBgz(self,newValue):
   #    self.bg[2][0] = newValue

   def get_array(self):
      xArray = np.concatenate((self.p,self.q,self.v,self.ba,self.bg),axis=0)
      return xArray

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
