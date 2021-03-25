import numpy as np

class States:
   def __init__(self,pMeters,qDegrees,vMetersPerSecond,baMetersPerSecondSquared,bgDegreesPerSecond,Pp,Pq,Pv,Pba,Pbg):
      #TODO need to make units clear and type (numpy array?)
      self.p = pMeters
      self.q = qDegrees
      self.v = vMetersPerSecond
      self.ba = baMetersPerSecondSquared
      self.bg = bgDegreesPerSecond

      self.P = np.array([[Pp.item(0),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],
                         [0.0,Pp.item(1),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],
                         [0.0,0.0,Pp.item(2),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],
                         [0.0,0.0,0.0,Pq.item(0),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],
                         [0.0,0.0,0.0,0.0,Pq.item(1),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],
                         [0.0,0.0,0.0,0.0,0.0,Pq.item(2),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],
                         [0.0,0.0,0.0,0.0,0.0,0.0,Pv.item(0),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],
                         [0.0,0.0,0.0,0.0,0.0,0.0,0.0,Pv.item(1),0.0,0.0,0.0,0.0,0.0,0.0,0.0],
                         [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,Pv.item(2),0.0,0.0,0.0,0.0,0.0,0.0],
                         [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,Pba.item(0),0.0,0.0,0.0,0.0,0.0],
                         [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,Pba.item(1),0.0,0.0,0.0,0.0],
                         [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,Pba.item(2),0.0,0.0,0.0],
                         [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,Pbg.item(0),0.0,0.0],
                         [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,Pbg.item(1),0.0],
                         [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,Pbg.item(2)]])

      self.dp = np.zeros((3,1))
      self.dq = np.zeros((3,1))
      self.dv = np.zeros((3,1))
      self.dba = np.zeros((3,1))
      self.dbg = np.zeros((3,1))

      self.At = np.zeros((15,15))