import numpy as np

class ImuMsg:
    def __init__(self,timeSeconds,accelerometersMetersPerSecondSquared,gyrosDegreesPerSecond):
        self.time = timeSeconds
        self.accelerometers = np.array([accelerometersMetersPerSecondSquared]).T
        self.gyros = np.array([gyrosDegreesPerSecond]).T

class GpsMsg:
    def __init__(self,positionEcefMeters,velocityEcefMetersPerSecond):
        self.positionEcef = np.array([positionEcefMeters]).T
        self.velocityEcef = np.array([velocityEcefMetersPerSecond]).T

class GpsCompassMsg:
    def __init__(self,headingDeg):
        self.heading = np.array([[headingDeg]]).T