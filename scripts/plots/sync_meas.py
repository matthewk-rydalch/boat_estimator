from rosbag_parser import Parser
from IPython.core.debugger import set_trace 
import matplotlib.pyplot as plt
import rosbag
import numpy as np
from collections import namedtuple

def main():
	data = Parser()
	filename = 'sync_meas.bag'
	bag = rosbag.Bag('/home/matt/data/px4flight/sim/' + filename)

	odom,imu,gps,gpsCompass = get_data(data, bag)
	get_north_data(odom,gps)
	# get_east_data(odom,gps)
	# get_down_data(odom,gps)


def get_data(data, bag):
	truth = data.get_boat_truth(bag)
	imu = data.get_boat_imu(bag)
	gps = data.get_boat_gps(bag)
	gpsCompass = data.get_gps_compass(bag)
	return truth,imu,gps,gpsCompass

def get_north_data(odom, gps):
	# odomTime = odom.time
	# odomN = np.array(odom.position[0])

	# gpsTime = gps.time
	# gpsN = np.array(gps.position[0])

	fig_num = 1
	set_trace()
	plot_2(fig_num, odom.time, odom.position[0], 'odom', gps.time, gps.position[0], 'gps')


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


if __name__ == '__main__':
	main()
