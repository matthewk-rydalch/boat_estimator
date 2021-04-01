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
                    gyrosRadiansPerSecond = [0.0,0.0,0.0]):
        self.time = timeSeconds
        self.accelerometers = np.array([accelerometersMetersPerSecondSquared]).T
        self.gyros = np.array([gyrosRadiansPerSecond]).T

    def __getitem__(self, key):
        switcher = {
            0: self.accelerometers[0][0],
            1: self.accelerometers[1][0],
            2: self.accelerometers[2][0],
            3: self.gyros[0][0],
            4: self.gyros[1][0],
            5: self.gyros[2][0],
        }
        return switcher.get(key, "invalid index")

    def __setitem__(self, key, newValue):
        switcher = {
            0: self.setAx,
            1: self.setAy,
            2: self.setAz,
            3: self.setGx,
            4: self.setGy,
            5: self.setGz,
        }
        func = switcher.get(key, "invalid index")
        func(newValue)

    def setAx(self, newValue):
        self.accelerometers[0][0] = newValue
    def setAy(self, newValue):
        self.accelerometers[1][0] = newValue
    def setAz(self, newValue):
        self.accelerometers[2][0] = newValue
    def setGx(self, newValue):
        self.gyros[0][0] = newValue
    def setGy(self, newValue):
        self.gyros[1][0] = newValue
    def setGz(self, newValue):
        self.gyros[2][0] = newValue
    def get_copy(self):
        imuMsgCopy = ImuMsg(np.copy(self.time),np.copy(np.squeeze(self.accelerometers)),np.copy(np.squeeze(self.gyros)))
        return imuMsgCopy

class GpsMsg:
    def __init__(self,positionEcefMeters = [0.0,0.0,0.0],velocityEcefMetersPerSecond = [0.0,0.0,0.0]):
        self.positionEcef = np.array([positionEcefMeters]).T
        self.velocityEcef = np.array([velocityEcefMetersPerSecond]).T

class GpsCompassMsg:
    def __init__(self,headingDeg = 0.0):
        self.heading = np.array([[headingDeg]]).T