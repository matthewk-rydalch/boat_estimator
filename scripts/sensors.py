import numpy as np

class ImuMsg:
    def __init__(self,timeS,accelerometers,gyros):
        self.timeS = timeS
        self.accelerometers = np.array([accelerometers]).T
        self.gyros = np.array([gyros]).T

class GpsMsg:
    def __init__(self,positionEcef,velocityEcef):
        self.positionEcef = np.array([positionEcef]).T
        self.velocityEcef = np.array([velocityEcef]).T

class GpsCompassMsg:
    def __init__(self,headingDeg):
        self.headingDeg = np.array([[headingDeg]]).T