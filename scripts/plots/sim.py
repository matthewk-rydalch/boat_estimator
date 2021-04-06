from rosbag_parser import Parser
from IPython.core.debugger import set_trace 
import matplotlib.pyplot as plt
import rosbag
import numpy as np
import navpy

def main():
	estRelPosTopic = '/rover2base_relPos'
	odomTopic = '/base_odom'
	truthTopic = '/dummyTopic'
	truePoseTopic = '/gazebo_base_pose'
	imuTopic = '/base/imu'
	ubloxRelPosTopic = '/p2u/rover/RelPos'
	gpsTopic = '/p2u/base/PosVelEcef'
	gpsCompassTopic = '/p2u/base/compass/RelPos'
	refLlaTopic = '/p2u/rover/PosVelEcef'
	data = Parser(estRelPosTopic,odomTopic,truthTopic,truePoseTopic,imuTopic,ubloxRelPosTopic,gpsTopic,gpsCompassTopic,refLlaTopic)
	filename = 'p2u.bag'
	bag = rosbag.Bag('/home/matt/data/boatLanding_sim/' + filename)

	ubloxRelPos,estRelPos,odom,gps,refLla,pose,imu = get_data(data, bag)
	gpsNed = ecef2ned(gps,refLla)
	# imuIntegrated = integrate_imu(imu)

	get_prn_data(ubloxRelPos,estRelPos)
	get_pre_data(ubloxRelPos,estRelPos)
	get_prd_data(ubloxRelPos,estRelPos)
	get_pn_data(odom,gpsNed)
	get_pe_data(odom,gpsNed)
	get_pd_data(odom,gpsNed)

	plt.show()

def get_data(data, bag):
	ubloxRelPos = data.get_ublox_relPos(bag)
	ubloxRelPos.time = ubloxRelPos.time - ubloxRelPos.time[0]
	ubloxRelPos.position = -ubloxRelPos.position
	
	estRelPos = data.get_estimated_relPos(bag)
	estRelPos.time = estRelPos.time - estRelPos.time[0]

	odom = data.get_odom(bag)
	odom.time = odom.time - odom.time[0]
	
	gps = data.get_gps(bag)
	gps.time = gps.time - gps.time[0]
	
	refLla = data.get_ref_lla(bag)
	refLla.lat = refLla.lat.item(0)
	refLla.lon = refLla.lon.item(0)
	refLla.alt = refLla.alt.item(0)

	pose = data.get_true_pose(bag)
	pose.time = pose.time - pose.time[0]

	imu = data.get_imu(bag)
	imu.time = imu.time - imu.time[0]

	return ubloxRelPos, estRelPos, odom, gps, refLla, pose, imu

def ecef2ned(gps,refLla):
	ecefOrigin1D = navpy.lla2ecef(refLla.lat,refLla.lon,refLla.alt)
	ecefOrigin = np.array([ecefOrigin1D]).T
	ecefPositionLocal = gps.position - ecefOrigin
	gpsPositionNed = navpy.ecef2ned(ecefPositionLocal,refLla.lat,refLla.lon,refLla.alt)
	gpsVelocityNed = navpy.ecef2ned(gps.velocity,refLla.lat,refLla.lon,refLla.alt)
	gpsNed = GpsNed(gps.time,gpsPositionNed,gpsVelocityNed)
	return gpsNed

# def integrate_imu(imu,pose):
# 	imuU = []
# 	imuV = []
# 	imuW = []
# 	imuX = []
# 	imuY = []
# 	imuZ = []
# 	imuPhi = []
# 	imuTheta = []
# 	imuPsi = []

# 	for i in range(len(imu.time)):
# 		if i == 0:
# 			dt = imu.time[1]-imu.time[0]
# 			imuU.append(0.0)
# 			imuV.append(0.0)
# 			imuW.append(0.0)
# 			imuX.append(pose.position[0][0] + imuU[-1]*dt)
# 			imuY.append(pose.position[1][0] + imuV[-1]*dt)
# 			imuZ.append(pose.position[2][0] + imuW[-1]*dt)
# 			imuPhi.append(imu.omega[0][i]*dt) #TODO: Need to add initial euler angles.  May need to integrate differently/propertly for angles.
# 			imuTheta.append(imu.omega[1][i]*dt)
# 			imuPsi.append(imu.omega[2][i]*dt)
# 		if i == 1:
# 			dt = imu.time[1]-imu.time[0]
# 			imuU.append(truth.velocity[0][0] + imu.accel[0][i]*dt)
# 			imuV.append(truth.velocity[1][0] + imu.accel[1][i]*dt)
# 			imuW.append(truth.velocity[2][0] + imu.accel[2][i]*dt)
# 			imuX.append(truth.position[0][0] + imuU[-1]*dt)
# 			imuY.append(truth.position[1][0] + imuV[-1]*dt)
# 			imuZ.append(truth.position[2][0] + imuW[-1]*dt)
# 			imuPhi.append(imu.omega[0][i]*dt) #TODO: Need to add initial euler angles.  May need to integrate differently/propertly for angles.
# 			imuTheta.append(imu.omega[1][i]*dt)
# 			imuPsi.append(imu.omega[2][i]*dt)
# 		else:
# 			dt = imu.time[i] - imu.time[i-1]
# 			imuU.append(imuU[-1] + imu.accel[0][i]*dt)
# 			imuV.append(imuV[-1] + imu.accel[1][i]*dt)
# 			imuW.append(imuW[-1] + imu.accel[2][i]*dt)
# 			imuX.append(imuX[-1] + imuU[-1]*dt)
# 			imuY.append(imuY[-1] + imuV[-1]*dt)
# 			imuZ.append(imuZ[-1] + imuW[-1]*dt)
# 			imuPhi.append(imuPhi[-1] + imu.omega[0][i]*dt)
# 			imuTheta.append(imuTheta[-1] + imu.omega[1][i]*dt)
# 			imuPsi.append(imuPsi[-1] + imu.omega[2][i]*dt)
# 	return ImuIntegrated(imu.time,imuX,imuY,imuZ,imuPhi,imuTheta,imuPsi,imuU,imuV,imuW)

def get_prn_data(ubloxRelPos, estRelPos):
	fig_num = 1
	plot_2(fig_num, ubloxRelPos.time, ubloxRelPos.position[0], 'ublox n', estRelPos.time, estRelPos.position[0], 'estimate n')

def get_pre_data(ubloxRelPos, estRelPos):
	fig_num = 2
	plot_2(fig_num, ubloxRelPos.time, ubloxRelPos.position[1], 'ublox e', estRelPos.time, estRelPos.position[1], 'estimate e')

def get_prd_data(ubloxRelPos, estRelPos):
	fig_num = 3
	plot_2(fig_num, ubloxRelPos.time, ubloxRelPos.position[2], 'ublox d', estRelPos.time, estRelPos.position[2], 'estimate d')

def get_pn_data(odom, gpsNed):
	fig_num = 4
	plot_2(fig_num, odom.time, odom.position[0], 'odom n', gpsNed.time, gpsNed.position[0], 'gps n')

def get_pe_data(odom, gpsNed):
	fig_num = 5
	plot_2(fig_num, odom.time, odom.position[1], 'odom e', gpsNed.time, gpsNed.position[1], 'gps e')

def get_pd_data(odom, gpsNed):
	fig_num = 6
	plot_2(fig_num, odom.time, odom.position[2], 'odom d', gpsNed.time, gpsNed.position[2], 'gps d')

def plot_2(fig_num, t_x, x, xlabel, t_y, y, ylabel):

	plt.figure(fig_num)
	plt.plot(t_x, x, label = xlabel)
	plt.plot(t_y, y, label = ylabel)
	plt.legend(loc = "upper right")

class GpsNed:
	def __init__(self,time,position,velocity):
		self.time = time
		self.position = position.T
		self.velocity = velocity.T

if __name__ == '__main__':
	main()
