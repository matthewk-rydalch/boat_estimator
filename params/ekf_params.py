import numpy as np

class EKFParams:
    def __init__(self):
        # self.p0 = np.zeros((3,1))
        # self.q0 = np.zeros((3,1))
        # self.v0 = np.zeros((3,1))
        self.p0 = np.array([[-2.2,1.0,3.0]]).T
        self.q0 = np.array([[0.0,0.0,0.0]]).T
        self.v0 = np.array([[0.0,-0.5,1.0]]).T
        self.ba0 = np.zeros((3,1))
        self.bg0 = np.zeros((3,1))

        self.pCov0 = np.array([[10.0,10.0,10.0]]).T
        self.qCov0 = np.array([[10.0,10.0,10.0]]).T
        self.vCov0 = np.array([[10.0,10.0,10.0]]).T
        self.baCov0 = np.array([[10.0,10.0,10.0]]).T
        self.bgCov0 = np.array([[10.0,10.0,10.0]]).T

        RtDiag = [0.00001,0.00001,0.00001, \
                  0.00001,0.00001,0.00001, \
                  0.00001,0.00001,0.00001, \
                  1.0,1.0,1.0, \
                  1.0,1.0,1.0]
        self.Rt = np.diag(RtDiag)
        # QtGpsDiag = [0.01,0.01,0.01,0.01,0.01,0.01]
        QtGpsDiag = [0.4,0.4,0.7,0.4,0.4,0.4]
        self.QtGps = np.diag(QtGpsDiag)
        self.QtGpsCompass = np.array([[0.02]])

        self.gravity = np.array([[0.0,0.0,9.81]]).T

