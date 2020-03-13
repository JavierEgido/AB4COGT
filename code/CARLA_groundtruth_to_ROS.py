#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2 
from sensor_msgs.msg import PointField
import sensor_msgs.point_cloud2 as pcl2

import numpy as np
import os
import sys
import glob
import argparse

classification_list = ["unknown",
			"Unknown_Small",
			"Unknown_Medium",
			"Unknown_Big",
			"Pedestrian",
			"Cyclist",
			"Car",
			"Truck",
			"Motorcycle",
			"Other_Vehicle",
			"Barrier",
			"Sign"]

class ros_publishers():

	
	def publisher(self, args):

		scene = args.scene
		directory = "../standalone/" + scene.zfill(4)
		pointcloud_list = glob.glob(directory + "/*.npy")
		pointcloud_list.sort()

		for pcl_file in pointcloud_list:

			points = np.fromfile(pcl_file).reshape(-1, 3)
			intensity = np.zeros(points.shape[0]).reshape([points.shape[0],1])
			points = np.hstack([points, intensity])
			
			header = Header()
			frame =  pcl_file[-10:-4]
			#print(frame)
			sys.stdout.write("\rSequence %s" % frame)
			sys.stdout.flush()
			header.frame_id = "ego_vehicle/lidar/lidar"
			header.seq = int(frame)

			# fill pcl msg
			fields = [PointField('x', 0, PointField.FLOAT32, 1),
				  PointField('y', 4, PointField.FLOAT32, 1),
				  PointField('z', 8, PointField.FLOAT32, 1),
				  PointField('i', 12, PointField.FLOAT32, 1)]
			pcl_msg = pcl2.create_cloud(header, fields, points)

			self.pub.publish(pcl_msg)
			self.rate.sleep()

		print("\nScene is completed\n")



	def listener(self):

		self.pub = rospy.Publisher("/carla/ego_vehicle/lidar/lidar/point_cloud", PointCloud2, queue_size=10)
		rospy.init_node('CarlaGroundtruthPlayer', anonymous=True)
		self.rate = rospy.Rate(1000)

if __name__ == '__main__':

	argparser = argparse.ArgumentParser(
		description='AB4COGT LiDAR Publisher')
	argparser.add_argument(
		'--scene','-s',
		default=0,
		help='Specify scene to be stored. By default, -s 0, scene 0000 is stored')
	args = argparser.parse_args()

	print("\n*** AB4COGT .npy to ROS Pointcloud2 publisher ***")
	ros_pbs = ros_publishers()
	ros_pbs.listener()
	ros_pbs.publisher(args)




