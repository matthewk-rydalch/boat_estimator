import rosbag
import pickle
from collections import namedtuple
import numpy as np
from geometry_msgs.msg import Pose

class Parser:
	def get_boat_odom(self, bag):
		sec = []
		nsec = []
		pn = []
		pe = []
		pd = []
		qx = []
		qy = []
		qz = []
		qw = []
		vx = []
		vy = []
		vz = []

		for topic, msg, t in bag.read_messages(topics=['/boat_odom']):
			sec.append(msg.header.stamp.secs)
			nsec.append(msg.header.stamp.nsecs)
			pn.append(msg.pose.pose.position.x)
			pe.append(msg.pose.pose.position.y)
			pd.append(msg.pose.pose.position.z)
			qx.append(msg.pose.pose.orientation.x)
			qy.append(msg.pose.pose.orientation.y)
			qz.append(msg.pose.pose.orientation.z)
			qw.append(msg.pose.pose.orientation.w)
			vx.append(msg.twist.twist.linear.x)
			vy.append(msg.twist.twist.linear.y)
			vz.append(msg.twist.twist.linear.z)

		return Odom(sec,nsec,pn,pe,pd,qx,qy,qz,qw,vx,vy,vz)

	def get_boat_imu(self, bag):
		sec = []
		nsec = []
		ax = []
		ay = []
		az = []
		wx = []
		wy = []
		wz = []

		for topic, msg, t in bag.read_messages(topics=['/boat/imu']):
			sec.append(msg.header.stamp.secs)
			nsec.append(msg.header.stamp.nsecs)
			ax.append(msg.linear_acceleration.x)
			ay.append(msg.linear_acceleration.y)
			az.append(msg.linear_acceleration.z)
			wx.append(msg.angular_velocity.x)
			wy.append(msg.angular_velocity.y)
			wz.append(msg.angular_velocity.z)

		return Imu(sec,nsec,ax,ay,az,wx,wy,wz)

	def get_boat_gps(self, bag):
		sec = []
		nsec = []
		px = []
		py = []
		pz = []
		vx = []
		vy = []
		vz = []

		for topic, msg, t in bag.read_messages(topics=['/boat/PosVelEcef']):
			sec.append(msg.header.stamp.secs)
			nsec.append(msg.header.stamp.nsecs)
			px.append(msg.position[0])
			py.append(msg.position[1])
			pz.append(msg.position[2])
			vx.append(msg.velocity[0])
			vy.append(msg.velocity[1])
			vz.append(msg.velocity[2])

		return Gps(sec,nsec,px,py,pz,vx,vy,vz)

class Odom:
	def __init__(self,sec,nsec,pn,pe,pd,qx,qy,qz,qw,vx,vy,vz):
		
		self.time = np.array(sec)+np.array(nsec)*1E-9
		self.position = np.array([pn,pe,pd])
		self.quat = np.array([qx,qy,qz,qw])
		self.velocity = np.array([vx,vy,vz])

class Imu:
	def __init__(self,sec,nsec,ax,ay,az,wx,wy,wz):
		
		self.time = np.array(sec)+np.array(nsec)*1E-9
		self.accel = np.array([ax,ay,az])
		self.omega = np.array([wx,wy,wz])

class Gps:
	def __init__(self,sec,nsec,px,py,pz,vx,vy,vz):
		
		self.time = np.array(sec)+np.array(nsec)*1E-9
		self.position = np.array([px,py,pz])
		self.velocity = np.array([vx,vy,vz])

if __name__ == '__main__':
    vals = main()
