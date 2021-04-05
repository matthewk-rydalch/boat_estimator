import rosbag
import pickle
from collections import namedtuple
import numpy as np
from geometry_msgs.msg import Pose

class Parser:
	def __init__(self,estRelPosTopic,odomTopic,truthTopic,imuTopic,ubloxRelPosTopic,gpsTopic,gpsCompassTopic,refLlaTopic):
		self.estRelPosTopic = estRelPosTopic
		self.odomTopic = odomTopic
		self.truthTopic = truthTopic
		self.imuTopic = imuTopic
		self.ubloxRelPosTopic = ubloxRelPosTopic
		self.gpsTopic = gpsTopic
		self.gpsCompassTopic = gpsCompassTopic
		self.refLlaTopic = refLlaTopic

	def get_ublox_relPos(self, bag):
		sec = []
		nsec = []
		pn = []
		pe = []
		pd = []
		pnHp = []
		peHp = []
		pdHp = []

		for topic, msg, t in bag.read_messages(topics=[self.ubloxRelPosTopic]):
			sec.append(msg.header.stamp.secs)
			nsec.append(msg.header.stamp.nsecs)
			pn.append(msg.relPosNED[0])
			pe.append(msg.relPosNED[1])
			pd.append(msg.relPosNED[2])
			pnHp.append(msg.relPosHPNED[0])
			peHp.append(msg.relPosHPNED[1])
			pdHp.append(msg.relPosHPNED[2])

		return RelPos(sec,nsec,pn,pe,pd,pnHp,peHp,pdHp)

	def get_odom(self, bag):
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

		for topic, msg, t in bag.read_messages(topics=[self.odomTopic]):
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

	def get_truth(self, bag):
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

		for topic, msg, t in bag.read_messages(topics=[self.truthTopic]):
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

	def get_imu(self, bag):
		sec = []
		nsec = []
		ax = []
		ay = []
		az = []
		wx = []
		wy = []
		wz = []

		for topic, msg, t in bag.read_messages(topics=[self.imuTopic]):
			sec.append(msg.header.stamp.secs)
			nsec.append(msg.header.stamp.nsecs)
			ax.append(msg.linear_acceleration.x)
			ay.append(msg.linear_acceleration.y)
			az.append(msg.linear_acceleration.z)
			wx.append(msg.angular_velocity.x)
			wy.append(msg.angular_velocity.y)
			wz.append(msg.angular_velocity.z)

		return Imu(sec,nsec,ax,ay,az,wx,wy,wz)

	def get_estimated_relPos(self, bag):
		sec = []
		nsec = []
		pn = []
		pe = []
		pd = []

		for topic, msg, t in bag.read_messages(topics=[self.estRelPosTopic]):
			sec.append(msg.header.stamp.secs)
			nsec.append(msg.header.stamp.nsecs)
			pn.append(msg.vector.x)
			pe.append(msg.vector.y)
			pd.append(msg.vector.z)

		return RelPos(sec,nsec,pn,pe,pd)

	def get_gps(self, bag):
		sec = []
		nsec = []
		px = []
		py = []
		pz = []
		vx = []
		vy = []
		vz = []

		for topic, msg, t in bag.read_messages(topics=[self.gpsTopic]):
			sec.append(msg.header.stamp.secs)
			nsec.append(msg.header.stamp.nsecs)
			px.append(msg.position[0])
			py.append(msg.position[1])
			pz.append(msg.position[2])
			vx.append(msg.velocity[0])
			vy.append(msg.velocity[1])
			vz.append(msg.velocity[2])

		return Gps(sec,nsec,px,py,pz,vx,vy,vz)

	def get_gps_compass(self, bag):
		sec = []
		nsec = []
		heading = []

		for topic, msg, t in bag.read_messages(topics=[self.gpsCompassTopic]):
			sec.append(msg.header.stamp.secs)
			nsec.append(msg.header.stamp.nsecs)
			heading.append(msg.relPosHeading)

		return GpsCompass(sec,nsec,heading)

	def get_ref_lla(self, bag):
		lat = []
		lon = []
		alt = []

		for topic, msg, t in bag.read_messages(topics=[self.refLlaTopic]):
			lat.append(msg.x)
			lon.append(msg.y)
			alt.append(msg.z)

		return refLla(lat,lon,alt)

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

class RelPos:
	def __init__(self,sec,nsec,pn,pe,pd,pnHp=0.0,peHp=0.0,pdHp=0.0):
		self.time = np.array(sec)+np.array(nsec)*1E-9
		
		pn = np.array(pn) + np.array(pnHp)
		pe = np.array(pe) + np.array(peHp)
		pd = np.array(pd) + np.array(pdHp)
		self.position = np.array([pn,pe,pd])

class Gps:
	def __init__(self,sec,nsec,px,py,pz,vx,vy,vz):
		
		self.time = np.array(sec)+np.array(nsec)*1E-9
		self.position = np.array([px,py,pz])
		self.velocity = np.array([vx,vy,vz])

class GpsCompass:
	def __init__(self,sec,nsec,heading):
		
		self.time = np.array(sec)+np.array(nsec)*1E-9
		self.heading = np.array([heading])

class refLla:
	def __init__(self,lat,lon,alt):
		self.lat = np.array([lat])
		self.lon = np.array([lon])
		self.alt = np.array([alt])
