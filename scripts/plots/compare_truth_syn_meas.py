from rosbag_parser import Parser
# from IPython.core.debugger import set_trace 
import matplotlib.pyplot as plt
import rosbag
import numpy as np
import navpy
from scipy.spatial.transform import Rotation as R

def main():
	estRelPosTopic = '/dummyTopic'
	odomTopic = '/base_odom'
	truthTopic = '/truth'
	truePoseTopic = '/dummyTopic'
	imuTopic = '/imu'
	ubloxRelPosTopic = '/dummyTopic'
	gpsTopic = '/gps'
	gpsCompassTopic = '/gps_compass'
	refLlaTopic = '/ref_lla'
	data = Parser(estRelPosTopic,odomTopic,truthTopic,truePoseTopic,imuTopic,ubloxRelPosTopic,gpsTopic,gpsCompassTopic,refLlaTopic)
	filename = 'syn_meas.bag'
	bag = rosbag.Bag('/home/matt/data/px4flight/sim/' + filename)

	odom,truth = get_data(data, bag)
	
	odomEuler = quat2euler(odom)
	truthEuler = quat2euler(truth)

	get_north_data(truth,odom)
	get_east_data(truth,odom)
	get_down_data(truth,odom)

	get_roll_data(truthEuler,odomEuler)
	get_pitch_data(truthEuler,odomEuler)	
	get_yaw_data(truthEuler,odomEuler)

	get_u_data(truth,odom)
	get_v_data(truth,odom)
	get_w_data(truth,odom)

	plt.show()

def get_data(data, bag):
	odom = data.get_odom(bag)
	truth = data.get_truth(bag)

	return odom,truth

def quat2euler(odom):
	quat = odom.quat.T
	r = R.from_quat(quat)
	eulerRad = r.as_euler('xyz').T
	return OdomEuler(odom.time,eulerRad[0],eulerRad[1],eulerRad[2])

def get_north_data(truth,odom):
	fig_num = 1
	plot_2(fig_num, truth.time, truth.position[0], 'truth_north', odom.time, odom.position[0], 'estimated_north')
	finalError = odom.position[0][-1] - truth.position[0][-1]
	print('north final error = ', finalError, ' meters')

def get_east_data(truth,odom):
	fig_num = 2
	plot_2(fig_num, truth.time, truth.position[1], 'truth_east', odom.time, odom.position[1], 'estimated_east')
	finalError = odom.position[1][-1] - truth.position[1][-1]
	print('east final error = ', finalError, ' meters')

def get_down_data(truth,odom):
	fig_num = 3
	plot_2(fig_num, truth.time, truth.position[2], 'truth_down', odom.time, odom.position[2], 'estimated_down')
	finalError = odom.position[2][-1] - truth.position[2][-1]
	print('down final error = ', finalError, ' meters')

def get_roll_data(truth,odom):
	fig_num = 4
	plot_2(fig_num, truth.time, truth.phi, 'truth_roll', odom.time, odom.phi, 'estimated_roll')
	finalError = (odom.phi[-1] - truth.phi[-1])*180.0/np.pi
	print('phi final error = ', finalError, ' deg')
	
def get_pitch_data(truth,odom):
	fig_num = 5
	plot_2(fig_num, truth.time, truth.theta, 'truth_pitch', odom.time, odom.theta, 'estimated_pitch')
	finalError = (odom.theta[-1] - truth.theta[-1])*180.0/np.pi
	print('theta final error = ', finalError, ' deg')

def get_yaw_data(truth,odom):
	fig_num = 6
	plot_2(fig_num, truth.time, truth.psi, 'truth_yaw', odom.time, odom.psi, 'estimated_yaw')
	finalError = (odom.psi[-1] - truth.psi[-1])*180.0/np.pi
	print('psi final error = ', finalError, ' deg')

def get_u_data(truth,odom):
	fig_num = 7
	plot_2(fig_num, truth.time, truth.velocity[0], 'truth_u', odom.time, odom.velocity[0], 'estimated_u')
	finalError = odom.velocity[0][-1] - truth.velocity[0][-1]
	print('u final error = ', finalError, ' m/s')

def get_v_data(truth,odom):
	fig_num = 8
	plot_2(fig_num, truth.time, truth.velocity[1], 'truth_v', odom.time, odom.velocity[1], 'estimated_v')
	finalError = odom.velocity[1][-1] - truth.velocity[1][-1]
	print('v final error = ', finalError, ' m/s')

def get_w_data(truth,odom):
	fig_num = 9
	plot_2(fig_num, truth.time, truth.velocity[2], 'truth_w', odom.time, odom.velocity[2], 'estimated_w')
	finalError = odom.velocity[2][-1] - truth.velocity[2][-1]
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

class OdomEuler:
	def __init__(self,time,phi,theta,psi):
		self.time = time
		self.phi = phi
		self.theta = theta
		self.psi = psi

if __name__ == '__main__':
	main()
