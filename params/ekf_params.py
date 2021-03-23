import numpy as np

class EKFParams:
    def __init__(self):
        self.p0 = np.zeros((3,1))
        self.q0 = np.zeros((3,1))
        self.v0 = np.zeros((3,1))
        self.ba0 = np.zeros((3,1))
        self.bg0 = np.zeros((3,1))

        self.pCov0 = np.array([[1.0,1.0,1.0]]).T
        self.qCov0 = np.array([[1.0,1.0,1.0]]).T
        self.vCov0 = np.array([[1.0,1.0,1.0]]).T
        self.baCov0 = np.array([[1.0,1.0,1.0]]).T
        self.bgCov0 = np.array([[1.0,1.0,1.0]]).T

        RtDiag = [1.0,1.0,1.0, \
                  1.0,1.0,1.0, \
                  1.0,1.0,1.0, \
                  1.0,1.0,1.0, \
                  1.0,1.0,1.0]
        self.Rt = np.diag(RtDiag)

