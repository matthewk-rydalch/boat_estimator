from rosbag_parser import Parser
# from IPython.core.debugger import set_trace 
import matplotlib.pyplot as plt
import rosbag
import numpy as np
from scipy.spatial.transform import Rotation as R

def main():
	params1OdomTopic = '/params1/boat_odom'
	params2OdomTopic = '/params2/boat_odom'
	truthTopic = '/truth'
	imuTopic = '/dummyTopic'
	gpsTopic = '/dummyTopic'
	gpsCompassTopic = '/dummyTopic'
	refLlaTopic = '/dummyTopic'
	params1Data = Parser(params1OdomTopic,truthTopic,imuTopic,gpsTopic,gpsCompassTopic,refLlaTopic)
	params2Data = Parser(params2OdomTopic,truthTopic,imuTopic,gpsTopic,gpsCompassTopic,refLlaTopic)
	filename = 'compare_params.bag'
	bag = rosbag.Bag('/home/matt/data/px4flight/sim/' + filename)

	odom1,odom2,truth = get_data(params1Data,params2Data,bag)
	
	odomEuler1 = quat2euler(odom1)
	odomEuler2 = quat2euler(odom2)
	truthEuler = quat2euler(truth)

	get_north_data(truth,odom1,odom2)
	get_east_data(truth,odom1,odom2)
	get_down_data(truth,odom1,odom2)

	get_roll_data(truthEuler,odomEuler1,odomEuler2)
	get_pitch_data(truthEuler,odomEuler1,odomEuler2)	
	get_yaw_data(truthEuler,odomEuler1,odomEuler2)

	get_u_data(truth,odom1,odom2)
	get_v_data(truth,odom1,odom2)
	get_w_data(truth,odom1,odom2)

	plt.show()

def get_data(data1, data2, bag):
	odom1 = data1.get_odom(bag)
	odom2 = data2.get_odom(bag)
	truth = data1.get_truth(bag)
	return odom1,odom2,truth

def quat2euler(odom):
	quat = odom.quat.T
	r = R.from_quat(quat)
	eulerRad = r.as_euler('xyz').T
	return OdomEuler(odom.time,eulerRad[0],eulerRad[1],eulerRad[2])

def get_north_data(truth,odom1,odom2):
	fig_num = 1
	plot_3(fig_num, truth.time, truth.position[0], 'truth_north', odom1.time, odom1.position[0], 'odom1_north', odom2.time, odom2.position[0], 'odom2_north')

def get_east_data(truth,odom1,odom2):
	fig_num = 2
	plot_3(fig_num, truth.time, truth.position[1], 'truth_east', odom1.time, odom1.position[1], 'odom1_east', odom2.time, odom2.position[1], 'odom2_down')

def get_down_data(truth,odom1,odom2):
	fig_num = 3
	plot_3(fig_num, truth.time, truth.position[2], 'truth_down', odom1.time, odom1.position[2], 'odom1_down', odom2.time, odom2.position[2], 'odom2_down')

def get_roll_data(truth,odom1,odom2):
	fig_num = 4
	plot_3(fig_num, truth.time, truth.phi, 'truth_roll', odom1.time, odom1.phi, 'odom1_roll', odom2.time, odom2.phi, 'odom2_roll')
	
def get_pitch_data(truth,odom1,odom2):
	fig_num = 5
	plot_3(fig_num, truth.time, truth.theta, 'truth_pitch', odom1.time, odom1.theta, 'odom1_pitch', odom2.time, odom2.theta, 'odom2_pitch')

def get_yaw_data(truth,odom1,odom2):
	fig_num = 6
	plot_3(fig_num, truth.time, truth.psi, 'truth_yaw', odom1.time, odom1.psi, 'odom1_yaw', odom2.time, odom2.psi, 'odom2_yaw')

def get_u_data(truth,odom1,odom2):
	fig_num = 7
	plot_3(fig_num, truth.time, truth.velocity[0], 'truth_u', odom1.time, odom1.velocity[0], 'odom1_u', odom2.time, odom2.velocity[0], 'odom2_u')

def get_v_data(truth,odom1,odom2):
	fig_num = 8
	plot_3(fig_num, truth.time, truth.velocity[1], 'truth_v', odom1.time, odom1.velocity[1], 'odom1_v', odom2.time, odom2.velocity[1], 'odom2_v')

def get_w_data(truth,odom1,odom2):
	fig_num = 9
	plot_3(fig_num, truth.time, truth.velocity[2], 'truth_w', odom1.time, odom1.velocity[2], 'odom1_w', odom2.time, odom2.velocity[2], 'odom2_w')

def plot_3(fig_num, t_x, x, xlabel, t_y, y, ylabel, t_z, z, zlabel):

	plt.figure(fig_num)
	plt.plot(t_x, x, label = xlabel)
	plt.plot(t_y, y, label = ylabel)
	plt.plot(t_z, z, label = zlabel)
	plt.legend(loc = "upper right")

class OdomEuler:
	def __init__(self,time,phi,theta,psi):
		self.time = time
		self.phi = phi
		self.theta = theta
		self.psi = psi

if __name__ == '__main__':
	main()
