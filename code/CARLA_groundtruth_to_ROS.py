#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2 
from sensor_msgs.msg import PointField
import sensor_msgs.point_cloud2 as pcl2

from datetime import datetime

import numpy as np
import os
import glob

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

	
	def publisher(self):
		#os.chdir("./lidar")
		pointcloud_list = glob.glob("./lidar3/*.npy")
		pointcloud_list.sort()
		try:
			for pcl_file in pointcloud_list:
				
					points = np.fromfile(pcl_file).reshape(-1, 3)
					intensity = np.zeros(points.shape[0]).reshape([points.shape[0],1])
					points = np.hstack([points, intensity])
					
					header = Header()
					#print("frame", pcl_file[8:14]) #for /lidar folder
					print("frame", pcl_file[9:15])  #for /lidar2 folder
					header.frame_id = "ego_vehicle/lidar/lidar"
					#header.stamp = rospy.Time.from_sec(float(datetime.strftime(dt, "%s.%f")))

					# fill pcl msg
					fields = [PointField('x', 0, PointField.FLOAT32, 1),
						  PointField('y', 4, PointField.FLOAT32, 1),
						  PointField('z', 8, PointField.FLOAT32, 1),
						  PointField('i', 12, PointField.FLOAT32, 1)]
					pcl_msg = pcl2.create_cloud(header, fields, points)

					self.pub.publish(pcl_msg)
					self.rate.sleep()

		except KeyboardInterrupt:
			stored_exception=sys.exc_info()

	def listener(self):

		self.pub = rospy.Publisher("/carla/ego_vehicle/lidar/lidar/point_cloud", PointCloud2, queue_size=10)
		rospy.init_node('CarlaGroundtruthPlayer', anonymous=True)
		self.rate = rospy.Rate(10)

if __name__ == '__main__':

	
	ros_pbs = ros_publishers()
	ros_pbs.listener()
	ros_pbs.publisher()




