import numpy as np

class States:
   def __init__(self,p,th,v,ba,bg,Px,Pth,Pv,Pba,Pbg):
      self.p = p
      self.th = th
      self.v = v
      self.ba = ba
      self.bg = bg

      self.P = np.array([[Px[0],0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],
                         [0.0,Px[1],0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],
                         [0.0,0.0,Px[2],0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],
                         [0.0,0.0,0.0,Pth[0],0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],
                         [0.0,0.0,0.0,0.0,Pth[1],0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],
                         [0.0,0.0,0.0,0.0,0.0,Pth[2],0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],
                         [0.0,0.0,0.0,0.0,0.0,0.0,Pv[0],0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],
                         [0.0,0.0,0.0,0.0,0.0,0.0,0.0,Pv[1],0.0,0.0,0.0,0.0,0.0,0.0,0.0],
                         [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,Pv[2],0.0,0.0,0.0,0.0,0.0,0.0],
                         [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,Pba[0],0.0,0.0,0.0,0.0,0.0],
                         [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,Pba[1],0.0,0.0,0.0,0.0],
                         [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,Pba[2],0.0,0.0,0.0],
                         [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,Pbg[0],0.0,0.0],
                         [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,Pbg[1],0.0],
                         [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,Pbg[2]]])

      self.dp = np.zeros((3,1))
      self.dth = np.zeros((3,1))
      self.dv = np.zeros((3,1))
      self.dba = np.zeros((3,1))
      self.dbg = np.zeros((3,1))

      self.A = np.zeros((15,15))