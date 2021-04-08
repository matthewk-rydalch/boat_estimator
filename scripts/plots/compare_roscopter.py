from rosbag_parser import Parser
from IPython.core.debugger import set_trace 
import matplotlib.pyplot as plt
import rosbag
import numpy as np
import navpy

def main():
	odomTopic = '/base_odom'
	truthTopic = '/dummyTopic'
	truePoseTopic = '/dummyTopic'
	imuTopic = '/boat/imu'
	ubloxRelPosTopic = '/rover/RelPos'
	baseGpsTopic = '/boat/PosVelEcef'
	roverGpsTopic = '/rover/PosVelEcef'
	rtkCompassTopic = '/boat/compass/RelPos'
	data = Parser(odomTopic,truthTopic,truePoseTopic,imuTopic,ubloxRelPosTopic,baseGpsTopic,roverGpsTopic,rtkCompassTopic)
	filename = 'compare_roscopter.bag'
	bag = rosbag.Bag('/home/matt/data/px4flight/sim/' + filename)

	timeOffsetEstAheadOfTruth = [1.25,-0.75]

	ubloxRelPos,estRelPos,estOdom,baseGps,refLla = get_data(data, bag, timeOffsetEstAheadOfTruth)
	baseGpsNed = ecef2ned(baseGps,refLla)

	get_pn_data(ubloxRelPos,estRelPos)
	get_pe_data(ubloxRelPos,estRelPos)
	get_pd_data(ubloxRelPos,estRelPos)
	get_ub_data(estOdom,baseGpsNed)
	get_vb_data(estOdom,baseGpsNed)
	get_wb_data(estOdom,baseGpsNed)
	get_attitude_data(estOdom)

	plt.show()

def get_data(data, bag, timeOffset):
	ubloxRelPos = data.get_ublox_relPos(bag)
	ubloxRelPos.time = ubloxRelPos.time - ubloxRelPos.time[0]
	ubloxRelPos.position = -ubloxRelPos.position
	
	estRelPos, estOdom = data.get_odom(bag)
	estRelPos.time = estRelPos.time - estRelPos.time[0] - timeOffset[0]
	estOdom.time = estOdom.time - estOdom.time[0] - timeOffset[1]
	
	baseGps = data.get_base_gps(bag)
	baseGps.time = baseGps.time - baseGps.time[0]

	roverGps,refLla = data.get_rover_gps(bag)
	refLla.lat = refLla.lat.item(0)
	refLla.lon = refLla.lon.item(0)
	refLla.alt = refLla.alt.item(0)

	return ubloxRelPos, estRelPos, estOdom, baseGps, refLla

def ecef2ned(gps,refLla):
	ecefOrigin1D = navpy.lla2ecef(refLla.lat,refLla.lon,refLla.alt)
	ecefOrigin = np.array([ecefOrigin1D]).T
	ecefPositionLocal = gps.position - ecefOrigin
	gpsPositionNed = navpy.ecef2ned(ecefPositionLocal,refLla.lat,refLla.lon,refLla.alt)
	gpsVelocityNed = navpy.ecef2ned(gps.velocity,refLla.lat,refLla.lon,refLla.alt)
	gpsNed = GpsNed(gps.time,gpsPositionNed,gpsVelocityNed)
	return gpsNed

def get_pn_data(ubloxRelPos, estRelPos):
	fig_num = 1
	plot_2(fig_num, ubloxRelPos.time, ubloxRelPos.position[0], 'ublox n', estRelPos.time, estRelPos.position[0], 'estimate n')
	finalError = ubloxRelPos.position[0][-1] - estRelPos.position[0][-1]
	print('relative north final error = ', finalError, ' meters')

def get_pe_data(ubloxRelPos, estRelPos):
	fig_num = 2
	plot_2(fig_num, ubloxRelPos.time, ubloxRelPos.position[1], 'ublox e', estRelPos.time, estRelPos.position[1], 'estimate e')
	finalError = ubloxRelPos.position[1][-1] - estRelPos.position[1][-1]
	print('relative east final error = ', finalError, ' meters')

def get_pd_data(ubloxRelPos, estRelPos):
	fig_num = 3
	plot_2(fig_num, ubloxRelPos.time, ubloxRelPos.position[2], 'ublox d', estRelPos.time, estRelPos.position[2], 'estimate d')
	finalError = ubloxRelPos.position[2][-1] - estRelPos.position[2][-1]
	print('relative down final error = ', finalError, ' meters')

def get_ub_data(odom, gpsNed):
	fig_num = 4
	plot_2(fig_num, odom.time, odom.velocity[0], 'odom u', gpsNed.time, gpsNed.velocity[0], 'gps u')
	finalError = odom.velocity[0][-1] - gpsNed.velocity[0][-1]
	print('north final error = ', finalError, ' meters')

def get_vb_data(odom, gpsNed):
	fig_num = 5
	plot_2(fig_num, odom.time, odom.velocity[1], 'odom v', gpsNed.time, gpsNed.velocity[1], 'gps v')
	finalError = odom.velocity[1][-1] - gpsNed.velocity[1][-1]
	print('East final error = ', finalError, ' meters')

def get_wb_data(odom, gpsNed):
	fig_num = 6
	plot_2(fig_num, odom.time, odom.velocity[2], 'odom w', gpsNed.time, gpsNed.velocity[2], 'gps w')
	finalError = odom.velocity[2][-1] - gpsNed.velocity[2][-1]
	print('down final error = ', finalError, ' meters')

def get_attitude_data(odom):
	fig_num = 7
	plot_3(fig_num, odom.time, odom.euler, 'phi deg', 'theta deg', 'psi deg')

def plot_2(fig_num, t_x, x, xlabel, t_y, y, ylabel):

	plt.figure(fig_num)
	plt.plot(t_x, x, label = xlabel)
	plt.plot(t_y, y, label = ylabel)
	plt.legend(loc = "upper right")

def plot_3(fig_num, t, x3, label1, label2, label3):
	plt.figure(fig_num)
	plt.plot(t,x3[:,0],label = label1)
	plt.plot(t,x3[:,1],label = label2)
	plt.plot(t,x3[:,2],label = label3)
	plt.legend(loc = "upper right")

class GpsNed:
	def __init__(self,time,position,velocity):
		self.time = time
		self.position = position.T
		self.velocity = velocity.T

if __name__ == '__main__':
	main()
