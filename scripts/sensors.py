import numpy as np

# class ImuMsg:
#     def __init__(self):
#         self.accelerometers = np.zeros((3,1))
#         self.gyros = np.zeros((3,1))

class GpsMsg:
    def __init__(self,positionEcef,velocityEcef):
        self.positionEcef = np.array([positionEcef]).T
        self.velocityEcef = np.array([velocityEcef]).T

# class GpsCompassMsg:
#     def __init__(self):
#         self.heading = 0.0