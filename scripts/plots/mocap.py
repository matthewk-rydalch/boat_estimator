from rosbag_parser import Parser
from IPython.core.debugger import set_trace 
import matplotlib.pyplot as plt
import rosbag
import numpy as np
import navpy

def main():
	odomTopic = '/base_odom'
	truthTopic = '/dummyTopic'
	mocapTopic = '/boat_landing_platform_ned'
	imuTopic = '/base/imu'
	ubloxRelPosTopic = '/m2u/rover/RelPos'
	baseGpsTopic = '/m2u/base/PosVelEcef'
	roverGpsTopic = '/m2u/rover/PosVelEcef'
	rtkCompassTopic = '/m2u/base/compass/RelPos'
	data = Parser(odomTopic,truthTopic,mocapTopic,imuTopic,ubloxRelPosTopic,baseGpsTopic,roverGpsTopic,rtkCompassTopic)
	filename = 'm2u_w_boat.bag'
	bag = rosbag.Bag('/home/matt/data/mocap/' + filename)

	timeOffset = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
	estRelPos,estOdom,m2uRelPos,m2uBaseGps,refLla,mocap = get_data(data, timeOffset, bag)
	m2uBaseGpsNed = ecef2ned(m2uBaseGps,refLla)

	get_pn_data(m2uRelPos,estRelPos,1)
	get_pe_data(m2uRelPos,estRelPos,2)
	get_pd_data(m2uRelPos,estRelPos,3)

	get_pn_data(mocap,estRelPos,4)
	get_pe_data(mocap,estRelPos,5)
	get_pd_data(mocap,estRelPos,6)

	get_phi_data(mocap,estOdom,7)
	get_theta_data(mocap,estOdom,8)
	get_psi_data(mocap,estOdom,9)

	get_ub_data(estOdom,m2uBaseGpsNed,10)
	get_vb_data(estOdom,m2uBaseGpsNed,11)
	get_wb_data(estOdom,m2uBaseGpsNed,12)

	plt.show()

def get_data(data, timeOffset, bag):
	estRelPos, estOdom = data.get_odom(bag)
	estRelPos.time += timeOffset[0]
	estOdom.time += timeOffset[1]

	# trueRelPos, trueOdom = data.get_truth(bag)
	# trueRelPos.time += timeOffset[2]
	# trueOdom.time += timeOffset[3]

	imu = data.get_imu(bag)
	imu.time += timeOffset[4]

	measuredRelPos = data.get_ublox_relPos(bag)
	measuredRelPos.time += timeOffset[5]

	baseGps = data.get_base_gps(bag)
	baseGps.time += timeOffset[6]

	roverGps,refLla = data.get_rover_gps(bag)
	roverGps.time += timeOffset[7]

	rtkCompass = data.get_rtk_compass(bag)
	rtkCompass.time += timeOffset[8]

	mocap = data.get_mocap(bag)
	mocap.time += timeOffset[9]

	return estRelPos, estOdom, measuredRelPos, baseGps, refLla, mocap

def ecef2ned(gps,refLla):
	ecefOrigin1D = navpy.lla2ecef(refLla.lat,refLla.lon,refLla.alt)
	ecefOrigin = np.array([ecefOrigin1D]).T
	ecefPositionLocal = gps.position - ecefOrigin
	gpsPositionNed = navpy.ecef2ned(ecefPositionLocal,refLla.lat,refLla.lon,refLla.alt)
	gpsVelocityNed = navpy.ecef2ned(gps.velocity,refLla.lat,refLla.lon,refLla.alt)
	gpsNed = GpsNed(gps.time,gpsPositionNed,gpsVelocityNed)
	return gpsNed

def get_pn_data(relPos, estRelPos, figNum):
	plot_2(figNum, relPos.time, -relPos.position[0], 'ublox n', estRelPos.time, estRelPos.position[0], 'estimate n')

def get_pe_data(relPos, estRelPos, figNum):
	plot_2(figNum, relPos.time, -relPos.position[1], 'ublox e', estRelPos.time, estRelPos.position[1], 'estimate e')

def get_pd_data(relPos, estRelPos, figNum):
	plot_2(figNum, relPos.time, -relPos.position[2], 'ublox d', estRelPos.time, estRelPos.position[2], 'estimate d')

def get_phi_data(mocap, estOdom, figNum):
	plot_2(figNum, mocap.time, mocap.euler[0], 'phi mocap deg', estOdom.time, estOdom.euler[0], 'phi est deg')

def get_theta_data(mocap, estOdom, figNum):
	plot_2(figNum, mocap.time, mocap.euler[1], 'theta mocap deg', estOdom.time, estOdom.euler[1], 'theta est deg')

def get_psi_data(mocap, estOdom, figNum):
	plot_2(figNum, mocap.time, mocap.euler[2], 'psi mocap deg', estOdom.time, estOdom.euler[2], 'psi est deg')

def get_ub_data(odom, gpsNed, figNum):
	plot_2(figNum, odom.time, odom.velocity[0], 'odom u', gpsNed.time, gpsNed.velocity[0], 'gps u')

def get_vb_data(odom, gpsNed, figNum):
	plot_2(figNum, odom.time, odom.velocity[1], 'odom v', gpsNed.time, gpsNed.velocity[1], 'gps v')

def get_wb_data(odom, gpsNed, figNum):
	plot_2(figNum, odom.time, odom.velocity[2], 'odom w', gpsNed.time, gpsNed.velocity[2], 'gps w')

def plot_2(figNum, t_x, x, xlabel, t_y, y, ylabel):
	plt.figure(figNum)
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
