import numpy as np

class TruthMsg:
    def __init__(self,timeSeconds = 0.0,positionMetersNed = [0.0,0.0,0.0],orientationRadNed = [0.0,0.0,0.0], \
                    velocityMetersPerSecondBody = [0.0,0.0,0.0],accelerationMetersPerSecondSquaredBody = [0.0,0.0,0.0], \
                    angularVelocityRadPerSecondBody = [0.0,0.0,0.0]):
        self.time = timeSeconds
        self.position = np.array([positionMetersNed]).T
        self.orientation = np.array([orientationRadNed]).T
        self.velocity = np.array([velocityMetersPerSecondBody]).T
        self.acceleration = np.array([accelerationMetersPerSecondSquaredBody]).T
        self.angularVelocity = np.array([angularVelocityRadPerSecondBody]).T

class ImuMsg:
    def __init__(self,timeSeconds = 0.0,accelerometersMetersPerSecondSquared = [0.0,0.0,0.0], \
                    gyrosDegreesPerSecond = [0.0,0.0,0.0]):
        self.time = timeSeconds
        self.accelerometers = np.array([accelerometersMetersPerSecondSquared]).T
        self.gyros = np.array([gyrosDegreesPerSecond]).T

class GpsMsg:
    def __init__(self,positionEcefMeters = [0.0,0.0,0.0],velocityEcefMetersPerSecond = [0.0,0.0,0.0]):
        self.positionEcef = np.array([positionEcefMeters]).T
        self.velocityEcef = np.array([velocityEcefMetersPerSecond]).T

class GpsCompassMsg:
    def __init__(self,headingDeg = 0.0):
        self.heading = np.array([[headingDeg]]).T