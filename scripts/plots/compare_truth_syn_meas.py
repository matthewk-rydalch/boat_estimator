from rosbag_parser import Parser
# from IPython.core.debugger import set_trace 
import matplotlib.pyplot as plt
import rosbag
import numpy as np
import navpy
from scipy.spatial.transform import Rotation as R

def main():
	odomTopic = '/base_odom'
	truthTopic = '/base_truth'
	imuTopic = '/base_imu'
	ubloxRelPosTopic = '/base2Rover_relPos'
	baseGpsTopic = '/base_gps'
	roverGpsTopic = '/rover_gps'
	rtkCompassTopic = '/base_rtk_compass'
	data = Parser(odomTopic,truthTopic,imuTopic,ubloxRelPosTopic,baseGpsTopic,roverGpsTopic,rtkCompassTopic)
	filename = 'syn_meas.bag'
	bag = rosbag.Bag('/home/matt/data/px4flight/sim/' + filename)

	timeOffset = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
	estRelPos,estOdom,trueRelPos,trueOdom = get_data(data, timeOffset, bag)

	get_north_data(trueRelPos,estRelPos)
	get_east_data(trueRelPos,estRelPos)
	get_down_data(trueRelPos,estRelPos)

	get_roll_data(trueOdom,estOdom)
	get_pitch_data(trueOdom,estOdom)	
	get_yaw_data(trueOdom,estOdom)

	get_u_data(trueOdom,estOdom)
	get_v_data(trueOdom,estOdom)
	get_w_data(trueOdom,estOdom)

	plt.show()

def get_data(data, timeOffset, bag):
	estRelPos, estOdom = data.get_odom(bag)
	estRelPos.time += timeOffset[0]
	estOdom.time += timeOffset[1]

	trueRelPos, trueOdom = data.get_truth(bag)
	trueRelPos.time += timeOffset[2]
	trueOdom.time += timeOffset[3]

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

	return estRelPos, estOdom, trueRelPos, trueOdom

def get_north_data(truth,est):
	fig_num = 1
	plot_2(fig_num, truth.time, truth.position[0], 'truth_north', est.time, est.position[0], 'estimated_north')
	finalError = est.position[0][-1] - truth.position[0][-1]
	print('north final error = ', finalError, ' meters')

def get_east_data(truth,est):
	fig_num = 2
	plot_2(fig_num, truth.time, truth.position[1], 'truth_east', est.time, est.position[1], 'estimated_east')
	finalError = est.position[1][-1] - truth.position[1][-1]
	print('east final error = ', finalError, ' meters')

def get_down_data(truth,est):
	fig_num = 3
	plot_2(fig_num, truth.time, truth.position[2], 'truth_down', est.time, est.position[2], 'estimated_down')
	finalError = est.position[2][-1] - truth.position[2][-1]
	print('down final error = ', finalError, ' meters')

def get_roll_data(truth,est):
	fig_num = 4
	plot_2(fig_num, truth.time, truth.euler[0], 'truth_roll', est.time, est.euler[0], 'estimated_roll')
	finalError = est.euler[0][-1] - truth.euler[0][-1]
	print('phi final error = ', finalError, ' deg')
	
def get_pitch_data(truth,est):
	fig_num = 5
	plot_2(fig_num, truth.time, truth.euler[1], 'truth_pitch', est.time, est.euler[1], 'estimated_pitch')
	finalError = est.euler[1][-1] - truth.euler[1][-1]
	print('theta final error = ', finalError, ' deg')

def get_yaw_data(truth,est):
	fig_num = 6
	plot_2(fig_num, truth.time, truth.euler[2], 'truth_yaw', est.time, est.euler[2], 'estimated_yaw')
	finalError = est.euler[2][-1] - truth.euler[2][-1]
	print('psi final error = ', finalError, ' deg')

def get_u_data(truth,est):
	fig_num = 7
	plot_2(fig_num, truth.time, truth.velocity[0], 'truth_u', est.time, est.velocity[0], 'estimated_u')
	finalError = est.velocity[0][-1] - truth.velocity[0][-1]
	print('u final error = ', finalError, ' m/s')

def get_v_data(truth,est):
	fig_num = 8
	plot_2(fig_num, truth.time, truth.velocity[1], 'truth_v', est.time, est.velocity[1], 'estimated_v')
	finalError = est.velocity[1][-1] - truth.velocity[1][-1]
	print('v final error = ', finalError, ' m/s')

def get_w_data(truth,est):
	fig_num = 9
	plot_2(fig_num, truth.time, truth.velocity[2], 'truth_w', est.time, est.velocity[2], 'estimated_w')
	finalError = est.velocity[2][-1] - truth.velocity[2][-1]
	print('w final error = ', finalError, ' m/s')

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

class ImuIntegrated:
	def __init__(self,time,x,y,z,phi,theta,psi,u,v,w):
		self.time = time
		self.position = [x,y,z]
		self.orientation = [phi,theta,psi]
		self.velocity = [u,v,w]

class RelPos:
	def __init__(self,time,position):
		self.time = time
		self.position = position

if __name__ == '__main__':
	main()
