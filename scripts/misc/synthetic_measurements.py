#!/usr/bin/env python3

import rospy

from ublox.msg import PosVelEcef

class SyntheticMeasurements:
    def __init__(self):
        self.roverLla = Vector3()
        self.refLlaSet = False
        self.truth_pub_ = rospy.Publisher('truth',Odometry,queue_size=5, latch=True)
        self.ref_lla_pub_ = rospy.Publisher('imu', Vector3,queue_size=5, latch=True)
        self.rover_lla_sub_ = rospy.Subscriber('posVelEcef', PosVelEcef, self.roverPVECallback,queue_size=5)

        while not rospy.is_shutdown():
            rospy.spin()

    def roverPVECallback(self,msg):
        if not self.refLlaSet: 
            self.roverLla.x = msg.lla[0]
            self.roverLla.y = msg.lla[1]
            self.roverLla.z = msg.lla[2]

            self.ref_lla_pub_.publish(self.roverLla)
            self.refLlaSet = True

if __name__ == '__main__':
    rospy.init_node('synthetic_measurements', anonymous=True)
    try:
        syncMeas = SyntheticMeasurements()
    except:
        rospy.ROSInterruptException
    pass