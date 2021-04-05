from rosbag_parser import Parser
from IPython.core.debugger import set_trace 
import matplotlib.pyplot as plt
import rosbag
import numpy as np
import navpy

def main():
	estRelPosTopic = '/boat_relPos'
	odomTopic = '/dummyTopic'
	truthTopic = '/dummyTopic'
	imuTopic = '/dummyTopic'
	ubloxRelPosTopic = '/rover/RelPos'
	gpsTopic = '/dummyTopic'
	gpsCompassTopic = '/dummyTopic'
	refLlaTopic = '/dummyTopic'
	data = Parser(estRelPosTopic,odomTopic,truthTopic,imuTopic,ubloxRelPosTopic,gpsTopic,gpsCompassTopic,refLlaTopic)
	filename = 'compare_relPos.bag'
	bag = rosbag.Bag('/home/matt/data/px4flight/sim/' + filename)

	ubloxRelPos,estRelPos = get_data(data, bag)
	# ubloxRelPos = convert_to_rover2boat(ubloxRelPosBase2Rover)
	
	get_north_data(ubloxRelPos,estRelPos)
	get_east_data(ubloxRelPos,estRelPos)
	get_down_data(ubloxRelPos,estRelPos)

	plt.show()

def get_data(data, bag):
	ubloxRelPos = data.get_ublox_relPos(bag)
	ubloxRelPos.time = ubloxRelPos.time - ubloxRelPos.time[0]
	ubloxRelPos.position = -ubloxRelPos.position
	
	estRelPos = data.get_estimated_relPos(bag)
	estRelPos.time = estRelPos.time - estRelPos.time[0]

	return ubloxRelPos, estRelPos

# def convert_to_rover2boat(base2Rover):
# 	base2Rover.pn = -base2Rover.pn
# 	base2Rover.pe = -base2Rover.pe
# 	base2Rover.pd = -base2Rover.pd #This is now rover2Base

# 	return base2Rover

def get_north_data(ubloxRelPos, estRelPos):
	fig_num = 1
	plot_2(fig_num, ubloxRelPos.time, ubloxRelPos.position[0], 'ublox', estRelPos.time, estRelPos.position[0], 'estimate')

def get_east_data(ubloxRelPos, estRelPos):
	fig_num = 2
	plot_2(fig_num, ubloxRelPos.time, ubloxRelPos.position[1], 'ublox', estRelPos.time, estRelPos.position[1], 'estimate')

def get_down_data(ubloxRelPos, estRelPos):
	fig_num = 3
	plot_2(fig_num, ubloxRelPos.time, ubloxRelPos.position[2], 'ublox', estRelPos.time, estRelPos.position[2], 'estimate')

def plot_2(fig_num, t_x, x, xlabel, t_y, y, ylabel):

	plt.figure(fig_num)
	plt.plot(t_x, x, label = xlabel)
	plt.plot(t_y, y, label = ylabel)
	plt.legend(loc = "upper right")

if __name__ == '__main__':
	main()
