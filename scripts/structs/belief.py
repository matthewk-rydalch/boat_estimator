import numpy as np

class Belief:
   def __init__(self,relPosNedMeters,vRoverNedMetersPerSecond,psiRad,vBodyMetersPerSecond,PCovariance):
      self.pr = relPosNedMeters                 #Relative Position from rover to base, NED meters
      self.vr = vRoverNedMetersPerSecond        #Velocity of the rover, NED m/s
      self.psi = psiRad                         #Yaw of the base, radians
      self.vb = vBodyMetersPerSecond            #velocity of the base, body m/s

      self.P = PCovariance

      # self.m = 10
      # self.n = 1

   # def get_copy(self):
   #    xCopy = Belief(np.copy(self.p),np.copy(self.vr),np.copy(self.psi),np.copy(self.vb),np.copy(self.P))
   #    return xCopy

   # def add_to_item(self,index,addedValue):
   #    switcher = {
   #    0: self.setPrn,
   #    1: self.setPre,
   #    2: self.setPrd,
   #    3: self.setVrn,
   #    4: self.setVre,
   #    5: self.setVrd,
   #    6: self.setPn,
   #    7: self.setPe,
   #    8: self.setPd,
   #    9: self.setPhi,
   #    10: self.setTheta,
   #    11: self.setPsi,
   #    12: self.setU,
   #    13: self.setV,
   #    14: self.setW,
   #    15: self.setBax,
   #    16: self.setBay,
   #    17: self.setBaz,
   #    18: self.setBgx,
   #    19: self.setBgy,
   #    20: self.setBgz
   #    }
   #    func = switcher.get(index, "invalid index")
   #    func(addedValue)
   # def setPrn(self,addedValue):
   #    self.pr[0][0] += addedValue
   # def setPre(self,addedValue):
   #    self.pr[1][0] += addedValue
   # def setPrd(self,addedValue):
   #    self.pr[2][0] += addedValue
   # def setVrn(self,addedValue):
   #    self.vr[0][0] += addedValue
   # def setVre(self,addedValue):
   #    self.vr[1][0] += addedValue
   # def setVrd(self,addedValue):
   #    self.vr[2][0] += addedValue
   # def setPn(self,addedValue):
   #    self.p[0][0] += addedValue
   # def setPe(self,addedValue):
   #    self.p[1][0] += addedValue
   # def setPd(self,addedValue):
   #    self.p[2][0] += addedValue
   # def setPhi(self,addedValue):
   #    self.q[0][0] += addedValue
   # def setTheta(self,addedValue):
   #    self.q[1][0] += addedValue
   # def setPsi(self,addedValue):
   #    self.q[2][0] += addedValue
   # def setU(self,addedValue):
   #    self.v[0][0] += addedValue
   # def setV(self,addedValue):
   #    self.v[1][0] += addedValue
   # def setW(self,addedValue):
   #    self.v[2][0] += addedValue
   # def setBax(self,addedValue):
   #    self.ba[0][0] += addedValue
   # def setBay(self,addedValue):
   #    self.ba[1][0] += addedValue
   # def setBaz(self,addedValue):
   #    self.ba[2][0] += addedValue
   # def setBgx(self,addedValue):
   #    self.bg[0][0] += addedValue
   # def setBgy(self,addedValue):
   #    self.bg[1][0] += addedValue
   # def setBgz(self,addedValue):
   #    self.bg[2][0] += addedValue
