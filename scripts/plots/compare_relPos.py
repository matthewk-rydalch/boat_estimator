from rosbag_parser import Parser
from IPython.core.debugger import set_trace 
import matplotlib.pyplot as plt
import rosbag
import numpy as np
import navpy

def main():
	estRelPosTopic = '/boat_relPos'
	odomTopic = '/boat_odom'
	truthTopic = '/dummyTopic'
	imuTopic = '/boat/imu'
	ubloxRelPosTopic = '/rover/RelPos'
	gpsTopic = '/boat/PosVelEcef'
	gpsCompassTopic = '/boat/compass/RelPos'
	refLlaTopic = '/ref_lla'
	data = Parser(estRelPosTopic,odomTopic,truthTopic,imuTopic,ubloxRelPosTopic,gpsTopic,gpsCompassTopic,refLlaTopic)
	filename = 'compare_roscopter.bag'
	bag = rosbag.Bag('/home/matt/data/px4flight/sim/' + filename)

	ubloxRelPos,estRelPos,odom,gps,refLla = get_data(data, bag)
	gpsNed = ecef2ned(gps,refLla)

	get_prn_data(ubloxRelPos,estRelPos)
	get_pre_data(ubloxRelPos,estRelPos)
	get_prd_data(ubloxRelPos,estRelPos)
	get_pn_data(ubloxRelPos,estRelPos)
	get_pe_data(ubloxRelPos,estRelPos)
	get_pd_data(ubloxRelPos,estRelPos)

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

	return ubloxRelPos, estRelPos, odom,gps,refLla

def ecef2ned(gps,refLla):
	ecefOrigin1D = navpy.lla2ecef(refLla.lat,refLla.lon,refLla.alt)
	ecefOrigin = np.array([ecefOrigin1D]).T
	ecefPositionLocal = gps.position - ecefOrigin
	gpsPositionNed = navpy.ecef2ned(ecefPositionLocal,refLla.lat,refLla.lon,refLla.alt)
	gpsVelocityNed = navpy.ecef2ned(gps.velocity,refLla.lat,refLla.lon,refLla.alt)
	gpsNed = GpsNed(gps.time,gpsPositionNed,gpsVelocityNed)
	return gpsNed

def get_prn_data(ubloxRelPos, estRelPos):
	fig_num = 1
	plot_2(fig_num, ubloxRelPos.time, ubloxRelPos.position[0], 'ublox n', estRelPos.time, estRelPos.position[0], 'estimate n')

def get_pre_data(ubloxRelPos, estRelPos):
	fig_num = 2
	plot_2(fig_num, ubloxRelPos.time, ubloxRelPos.position[1], 'ublox n', estRelPos.time, estRelPos.position[1], 'estimate n')

def get_prd_data(ubloxRelPos, estRelPos):
	fig_num = 3
	plot_2(fig_num, ubloxRelPos.time, ubloxRelPos.position[2], 'ublox n', estRelPos.time, estRelPos.position[2], 'estimate n')

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
