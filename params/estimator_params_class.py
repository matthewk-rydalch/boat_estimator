import numpy as np
import rospy

class EstimatorParams:
    def __init__(self):
        self.pr0 = np.array([rospy.get_param('~pr0',[0.0,0.0,0.0])]).T
        self.vr0 = np.array([rospy.get_param('~vr0',[0.0,0.0,0.0])]).T
        self.p0 = np.array([rospy.get_param('~p0',[0.0,0.0,0.0])]).T
        self.q0 = np.array([rospy.get_param('~q0',[0.0,0.0,0.0])]).T
        self.v0 = np.array([rospy.get_param('~v0',[0.0,0.0,0.0])]).T
        self.ba0 = np.array([rospy.get_param('~ba0',[0.0,0.0,0.0])]).T
        self.bg0 = np.array([rospy.get_param('~bg0',[0.0,0.0,0.0])]).T

        sigma0 = rospy.get_param('~sigma0',[3.0,3.0,3.0,3.0,3.0,3.0,3.0,3.0,3.0,3.0,3.0,3.0,3.0,3.0,3.0,3.0,3.0,3.0,3.0,3.0,3.0])
        self.P0 = np.diag([
                    sigma0[0]**2,  # prn
                    sigma0[1]**2,  # pre
                    sigma0[2]**2,  # prd
                    sigma0[3]**2,  # vrn
                    sigma0[4]**2,  # vre
                    sigma0[5]**2,  # vrd
                    sigma0[6]**2,  # pn
                    sigma0[7]**2,  # pe
                    sigma0[8]**2,  # pd
                    np.radians(sigma0[9])**2,  # phi
                    np.radians(sigma0[10])**2,  # theta
                    np.radians(sigma0[11])**2,  # psi
                    sigma0[12]**2,  # u
                    sigma0[13]**2,  # v
                    sigma0[14]**2,  # w
                    sigma0[15]**2, #bax
                    sigma0[16]**2, #bay
                    sigma0[17]**2, #baz
                    np.radians(sigma0[18])**2,  # bgx
                    np.radians(sigma0[19])**2,  # bgy
                    np.radians(sigma0[20])**2,  # bgz
                   ])

        sigmaProcess = rospy.get_param('~sigmaProcess',[0.0003,0.0003,0.0003,0.0003,0.0003,0.0003,0.0003,0.0003,0.0003,0.0003,0.0003,0.0003,0.0003,0.0003,0.0003,1.0,1.0,1.0,1.0,1.0,1.0])
        self.RProcess = 1.0 * np.diag([
                        sigmaProcess[0]**2,  # prn
                        sigmaProcess[1]**2,  # pre
                        sigmaProcess[2]**2,  # prd
                        sigmaProcess[3]**2,  # vrn
                        sigmaProcess[4]**2,  # vre
                        sigmaProcess[5]**2,  # vrd
                        sigmaProcess[6]**2,  # pn
                        sigmaProcess[7]**2,  # pe
                        sigmaProcess[8]**2,  # pd
                        sigmaProcess[9]**2,  # phi
                        sigmaProcess[10]**2,  # theta
                        sigmaProcess[11]**2,  # psi
                        sigmaProcess[12]**2,  # u
                        sigmaProcess[13]**2,  # v
                        sigmaProcess[14]**2,  # w
                        sigmaProcess[15]**2,  # bax
                        sigmaProcess[16]**2,  # bay
                        sigmaProcess[17]**2,  # baz
                        sigmaProcess[18]**2,  # bgx
                        sigmaProcess[19]**2,  # bgy
                        sigmaProcess[20]**2,  # bgz
                        ])

        sigmaImu = rospy.get_param('~sigmaImu',[0.0003,0.0003,0.0003,0.0003,0.0003,0.0003])
        self.RImu = 1.0 * np.diag([
                        sigmaImu[0]**2,  # ax
                        sigmaImu[1]**2,  # ay
                        sigmaImu[2]**2,  # az
                        sigmaImu[3]**2,  # p
                        sigmaImu[4]**2,  # q
                        sigmaImu[5]**2,  # r
                        ])

        sigmaGps = rospy.get_param('~sigmaGps',[0.4,0.7,0.4,0.02,0.04,0.02])
        self.QtRtk = np.diag([
                    sigmaGps[3]**2, # rtkN
                    sigmaGps[3]**2, # rtkE
                    sigmaGps[4]**2, # rtkD
                    ])

        self.QtGpsVelocity = np.diag([
                    sigmaGps[2]**2, # gpsVelocityN
                    sigmaGps[2]**2, # gpsVelocityE
                    sigmaGps[2]**2, # gpsVelocityD
                    ])

        self.QtGps = np.diag([
                    sigmaGps[0]**2, # gpsN
                    sigmaGps[0]**2, # gpsE
                    sigmaGps[1]**2, # gpsD
                    sigmaGps[2]**2, # gpsU
                    sigmaGps[2]**2, # gpsV
                    sigmaGps[2]**2, # gpsW
                    ])

        self.QtRtkCompass = np.array([[sigmaGps[5]**2]]) # rtk compass heading

        self.gravity = np.array([rospy.get_param('~gravity',[0.0,0.0,9.81])]).T

        self.kp = rospy.get_param('~compFiltKp',0.1)

