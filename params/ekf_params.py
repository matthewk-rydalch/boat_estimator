import numpy as np
import rospy

class EKFParams:
    def __init__(self):
        self.p0 = np.array([rospy.get_param('~p0',[0.0,0.0,0.0])]).T
        self.q0 = np.array([rospy.get_param('~q0',[0.0,0.0,0.0])]).T
        self.v0 = np.array([rospy.get_param('~v0',[0.0,0.0,0.0])]).T
        self.ba0 = np.array([rospy.get_param('~ba0',[0.0,0.0,0.0])]).T
        self.bg0 = np.array([rospy.get_param('~bg0',[0.0,0.0,0.0])]).T
        # print('p0 = ', self.p0)
        # print('q0 = ', self.q0)
        # print('v0 = ', self.v0)
        # print('ba0 = ', self.ba0)
        # print('bg0 = ', self.bg0)

        sigma0 = rospy.get_param('~sigma0',[3.0,3.0,3.0,3.0,3.0,3.0,3.0,3.0,3.0,3.0,3.0,3.0,3.0,3.0,3.0])
        self.P0 = np.diag([
                    sigma0[0]**2,  # pn
                    sigma0[1]**2,  # pe
                    sigma0[2]**2,  # pd
                    np.radians(sigma0[3])**2,  # phi
                    np.radians(sigma0[4])**2,  # theta
                    np.radians(sigma0[5])**2,  # psi
                    sigma0[6]**2,  # u
                    sigma0[7]**2,  # v
                    sigma0[8]**2,  # w
                    sigma0[9]**2, #bax
                    sigma0[10]**2, #bay
                    sigma0[11]**2, #baz
                    np.radians(sigma0[12])**2,  # bgx
                    np.radians(sigma0[13])**2,  # bgy
                    np.radians(sigma0[14])**2,  # bgz
                   ])
        # print('P0 = ', self.P0)

        sigmaProcess = rospy.get_param('~sigmaProcess',[0.0003,0.0003,0.0003,0.0003,0.0003,0.0003,0.0003,0.0003,0.0003,1.0,1.0,1.0,1.0,1.0,1.0])
        self.RProcess = 1.0 * np.diag([
                        sigmaProcess[0]**2,  # pn
                        sigmaProcess[1]**2,  # pe
                        sigmaProcess[2]**2,  # pd
                        sigmaProcess[3]**2,  # phi
                        sigmaProcess[4]**2,  # theta
                        sigmaProcess[5]**2,  # psi
                        sigmaProcess[6]**2,  # u
                        sigmaProcess[7]**2,  # v
                        sigmaProcess[8]**2,  # w
                        sigmaProcess[9]**2,  # bax
                        sigmaProcess[10]**2,  # bay
                        sigmaProcess[11]**2,  # baz
                        sigmaProcess[12]**2,  # bgx
                        sigmaProcess[13]**2,  # bgy
                        sigmaProcess[14]**2,  # bgz
                        ])
        # print('RProcess = ', self.RProcess)

        sigmaImu = rospy.get_param('~sigmaImu',[0.0003,0.0003,0.0003,0.0003,0.0003,0.0003])
        self.RImu = 1.0 * np.diag([
                        sigmaImu[0]**2,  # ax
                        sigmaImu[1]**2,  # ay
                        sigmaImu[2]**2,  # az
                        sigmaImu[3]**2,  # p
                        sigmaImu[4]**2,  # q
                        sigmaImu[5]**2,  # r
                        ])
        # print('RImu = ', self.RImu)

        sigmaGps = rospy.get_param('~sigmaGps',[0.4,0.7,0.4,0.02])
        self.QtGps = np.diag([
                    sigmaGps[0]**2, # gpsN
                    sigmaGps[0]**2, # gpsE
                    sigmaGps[1]**2, # gpsD
                    sigmaGps[2]**2, # gpsU
                    sigmaGps[2]**2, # gpsV
                    sigmaGps[2]**2, # gpsW
                    ])
        # print('QtGps = ', self.QtGps)

        self.QtGpsCompass = np.array([[sigmaGps[3]**2]]) # rtk compass heading
        # print('QtGpsCompass = ', self.QtGpsCompass)

        self.gravity = np.array([rospy.get_param('~gravity',[0.0,0.0,9.81])]).T
        # print('gravity = ', self.gravity)

        self.kp = rospy.get_param('~compFiltKp',0.1)
        self.ki = rospy.get_param('~compFiltKi',0.0)
        # print('kp = ', self.kp)
        # print('ki = ', self.ki)

