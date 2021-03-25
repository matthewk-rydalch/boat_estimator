from rosbag_parser import Parser
from rosbag_parser import Gps
from IPython.core.debugger import set_trace 
import matplotlib.pyplot as plt
import rosbag
import numpy as np
from collections import namedtuple
import navpy

def main():
	odomTopic = '/odom'
	truthTopic = '/truth'
	imuTopic = '/imu'
	gpsTopic = '/gps'
	gpsCompassTopic = '/gps_compass'
	refLlaTopic = '/ref_lla'
	data = Parser(odomTopic,truthTopic,imuTopic,gpsTopic,gpsCompassTopic,refLlaTopic)
	filename = 'syn_meas.bag'
	bag = rosbag.Bag('/home/matt/data/px4flight/sim/' + filename)

	truth,imu,gps,gpsCompass,refLla = get_data(data, bag)
	imuIntegrated = integrateImu(imu,truth)
	gpsNed = ecef2ned(gps,refLla)
	get_north_data(truth,imuIntegrated)
	# get_east_data(odom,gps)
	# get_down_data(odom,gps)


def get_data(data, bag):
	truth = data.get_truth(bag)
	imu = data.get_imu(bag)
	gps = data.get_gps(bag)
	gpsCompass = data.get_gps_compass(bag)
	refLla = data.get_ref_lla(bag)
	return truth,imu,gps,gpsCompass,refLla

def integrateImu(imu,truth):
	imuU = []
	imuV = []
	imuW = []
	imuX = []
	imuY = []
	imuZ = []
	imuPhi = []
	imuTheta = []
	imuPsi = []

	for i in range(len(imu.time)):
		if i == 0:
			dt = imu.time[1]-imu.time[0]
			imuU.append(truth.velocity[0][0] + imu.accel[0][i]*dt)
			imuV.append(truth.velocity[1][0] + imu.accel[1][i]*dt)
			imuW.append(truth.velocity[2][0] + imu.accel[2][i]*dt)
			imuX.append(truth.position[0][0] + imuU[-1]*dt)
			imuY.append(truth.position[1][0] + imuV[-1]*dt)
			imuZ.append(truth.position[2][0] + imuW[-1]*dt)
			imuPhi.append(imu.omega[0][i]*dt) #TODO: Need to add initial euler angles.  May need to integrate differently/propertly for angles.
			imuTheta.append(imu.omega[1][i]*dt)
			imuPsi.append(imu.omega[2][i]*dt)
		else:
			dt = imu.time[i] - imu.time[i-1]
			imuU.append(imuU[-1] + imu.accel[0][i]*dt)
			imuV.append(imuV[-1] + imu.accel[1][i]*dt)
			imuW.append(imuW[-1] + imu.accel[2][i]*dt)
			imuX.append(imuX[-1] + imuU[-1]*dt)
			imuY.append(imuY[-1] + imuV[-1]*dt)
			imuZ.append(imuZ[-1] + imuW[-1]*dt)
			imuPhi.append(imuPhi[-1] + imu.omega[0][i]*dt)
			imuTheta.append(imuTheta[-1] + imu.omega[1][i]*dt)
			imuPsi.append(imuPsi[-1] + imu.omega[2][i]*dt)
	return ImuIntegrated(imu.time,imuX,imuY,imuZ,imuPhi,imuTheta,imuPsi,imuU,imuV,imuW)

def ecef2ned(gps,refLla):
	ecefOrigin1D = navpy.lla2ecef(refLla.lat,refLla.lon,refLla.alt)
	ecefOrigin = np.array([ecefOrigin1D]).T
	ecefPositionLocal = gps.position - ecefOrigin
	gpsPositionNed = navpy.ecef2ned(ecefPositionLocal,refLla.lat,refLla.lon,refLla.alt)
	gpsVelocityNed = navpy.ecef2ned(gps.velocity,refLla.lat,refLla.lon,refLla.alt)
	gpsNed = GpsNed(gps.time,gpsPositionNed,gpsVelocityNed)
	return gpsNed

def get_north_data(truth, imuIntegrated):

	fig_num = 1
	plot_2(fig_num, truth.time, truth.position[0], 'truth', imuIntegrated.time, imuIntegrated.position[0], 'imu')


def get_east_data(odom, gps):
	odomTime = odom.time
	odomE = np.array(odom.position[1])

	gpsTime = gps.time
	gpsE = np.array(gps.position[1])

	fig_num = 2
	plot_2(fig_num, odomTime, odomE, 'odom', gpsTime, gpsE, 'gps')


def get_down_data(odom, gps):
	odomTime = odom.time
	odomD = np.array(odom.position[2])

	gpsTime = gps.time
	gpsD = np.array(gps.position[2])

	fig_num = 3
	plot_2(fig_num, odomTime, odomD, 'odom', gpsTime, gpsD, 'gps')

def plot_2(fig_num, t_x, x, xlabel, t_y, y, ylabel):

	plt.figure(fig_num)
	plt.plot(t_x, x, label = xlabel)
	plt.plot(t_y, y, label = ylabel)
	plt.legend(loc = "upper right")
	plt.show()

class GpsNed:
	def __init__(self,time,position,velocity):
		self.time = time
		self.position = position.T
		self.velocity = velocity.T

class ImuIntegrated:
	def __init__(self,time,x,y,z,phi,theta,psi,u,v,w):
		self.time = time
		self.position = [x,y,z]
		self.orientation = [phi,theta,psi]
		self.velocity = [u,v,w]

if __name__ == '__main__':
	main()
