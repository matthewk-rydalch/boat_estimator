import numpy as np

class Belief:
   def __init__(self,relPosNedMeters,vRoverNedMetersPerSecond,psiRad,vBodyMetersPerSecond,PCovariance):
      #already numpy arrays from estimator params class
      self.p = relPosNedMeters                 #Relative Position from rover to base, NED meters
      self.vr = vRoverNedMetersPerSecond        #Velocity of the rover, NED m/s
      self.psi = psiRad                         #Yaw of the base, radians
      self.vb = vBodyMetersPerSecond            #velocity of the base, body m/s

      self.P = PCovariance

      self.m = 10
      self.n = 1