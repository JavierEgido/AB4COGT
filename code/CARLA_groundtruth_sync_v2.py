#!/usr/bin/env python

import rospy
import message_filters
from derived_object_msgs.msg import ObjectArray
from carla_msgs.msg import CarlaEgoVehicleInfo
from sensor_msgs.msg import PointCloud2 
import sensor_msgs.point_cloud2
from tf.transformations import euler_from_quaternion

import numpy as np
from scipy.spatial.distance import euclidean
import os
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

class ros_callbacks():
	
	def callback(self, data, pointcloud):

		#Firstly, find ego_vehicle position
		for obj1 in range(len(data.objects)):
			identity 	= data.objects[obj1].id
			if identity == self.ego_vehicle_id:
				xyz_ego     = data.objects[obj1].pose.position
				quat_ego    = data.objects[obj1].pose.orientation
				quaternion  = np.array((quat_ego.x, quat_ego.y, quat_ego.z, quat_ego.w))
				break
		location_ego = [xyz_ego.x, xyz_ego.y, xyz_ego.z]
		heading_ego  = euler_from_quaternion(quaternion)[2]

		frame = self.sequence
		print("Frame", frame)

		for obj in range(len(data.objects)):

			identity = data.objects[obj].id


			if identity != self.ego_vehicle_id:
							
				xyz = data.objects[obj].pose.position
				location = [xyz.x, xyz.y, xyz.z]

				#Only store groundtruth of objects in Lidar range
				if euclidean(location, location_ego) < 50: #Compare to LiDAR range
					#Get data from object topic
					quat_xyzw 	= data.objects[obj].pose.orientation
					lhw 		= data.objects[obj].shape.dimensions
					label 		= classification_list[data.objects[obj].classification]

					#Calculate heading and alpha (obs_angle)
					quaternion 	= np.array((quat_xyzw.x, quat_xyzw.y, quat_xyzw.z, quat_xyzw.w))
					heading 	= euler_from_quaternion(quaternion)[2]# - np.pi/2
					beta = np.arctan2(xyz.x - xyz_ego.x, xyz_ego.y - xyz.y)
					obs_angle = ( (heading) + (beta) )

					#Print data to .txt file
					hwl  = [lhw[1], lhw[2], lhw[0]]
					location_CARLA = (np.asarray(location) - np.asarray(location_ego)).tolist()
					R = self.rotz(-np.pi/2) #CARLA to LiDAR coordinates
					location_local = np.dot(R, location_CARLA)
					heading_local  = heading - heading_ego

					if (args.front and location_local[0]>0) or not args.front:

						box = [0,0,0,0]
						out_data= [frame] + [identity] + [label] + [0, 0] + [obs_angle] + box + hwl + list(location_local) + [heading_local]
						data_str=[]
						for param in out_data:
							data_str.append(str(param))
							data_str.append(" ")

						self.label_file.writelines(data_str)
						self.label_file.writelines("\n")


		if args.lidar:
			points = []
			for point in sensor_msgs.point_cloud2.read_points(pointcloud, skip_nans=True):
				points.extend(point)
			points = np.asarray(points).reshape([3,-1])
			filename = "../standalone/" + args.scene.zfill(4) + '/' + str(frame).zfill(6) + ".npy"
			points.tofile(filename)

		self.sequence += 1

	def ego_vehicle_callback(self, data):
		print("Receiving ego_vehicle")
		self.ego_vehicle_id = data.id

	def rotz(self, t):
		''' Rotation about the z-axis. '''
		c = np.cos(t)
		s = np.sin(t)
		return np.array([[c,  -s,  0],
			         [s,   c,  0],
			         [0,   0,  1]])
	

	def listener(self, args):

		self.sequence = 0
		self.args = args
		if not args.lidar: pointcloud = None

		txtfile = '../standalone/' + args.scene.zfill(4) + '.txt'
		if os.path.exists(txtfile): os.remove(txtfile)
		self.label_file = open(txtfile, "a")

		rospy.init_node('CarlaGroundtruthRecorder', anonymous=True)

		ego_vehicle_subs = rospy.Subscriber("/carla/ego_vehicle/vehicle_info", CarlaEgoVehicleInfo, self.ego_vehicle_callback)

		if args.lidar:
			lidar_sub = message_filters.Subscriber('/carla/ego_vehicle/lidar/lidar/point_cloud', PointCloud2)
			obj_sub = message_filters.Subscriber("/carla/objects", ObjectArray)

			ts = message_filters.TimeSynchronizer([obj_sub, lidar_sub], 10)
			ts.registerCallback(self.callback)

		else:
			self.pointcloud = None
			obj_sub = rospy.Subscriber("/carla/objects", ObjectArray, self.callback, [self, self.pointcloud])

		rospy.spin()

if __name__ == '__main__':

	argparser = argparse.ArgumentParser(
		description='AB4COGT ground-truth collector')
	argparser.add_argument(
		'--lidar','-v', '--velodyne', '-l',
		default=0,
		help='If set to 1, LiDAR will be stored in ./LiDAR')
	argparser.add_argument(
		'--front','-f',
		default=0,
		help='If set to 1, only forward objects are recorded')
	argparser.add_argument(
		'--scene','-s',
		default=0,
		help='Specify scene to be stored. By default, -s 0, scene 0000 is stored')

	args = argparser.parse_args()

	ros_cbs = ros_callbacks()
	ros_cbs.listener(args)



