#!/usr/bin/env python
import sys
import rospy
import numpy as np
import math
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2, LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Twist
from tf.transformations import euler_from_quaternion
import resource

class Movement:
	# publishers
	pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
	heading_pub = rospy.Publisher('heading', String, queue_size=1)

	# Global variables
	scan_min = 0

	x_delt = 0
	theta_delt = 0

	stored_x = 0
	stored_y = 0
	stored_theta = 0
	heading = ""

	# Create empty Twist message
	new_twist = Twist()

	# angles to round to for relative moves
	rounder_angles = [-3.14,-2.355,-1.57,-0.785,0,0.785,1.57,2.355,3.14]

	# Lookup dict to map direction to global angle in environment
	directions = {"N":1.57,"NE":0.785,"E":0,"SE":-0.785,"S":-1.57,"SW":-2.355,"W":3.14,"NW":2.355}
	
	def _odom_callback(self, data):
		""" Callback to get pose data of the robot and turn quaternion data into
		Euler angles

		Args:
			data: ros message of type nav_msgs/Odometry

		Returns:
			None

		"""
		orientation_q = data.pose.pose.orientation
		
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		
		(self.roll, self.pitch, self.yaw) = euler_from_quaternion (orientation_list)
		self.x_pos = data.pose.pose.position.x
		self.y_pos = data.pose.pose.position.y
		self.z_pos = data.pose.pose.position.z
	
	def _front_laser_callback(self, data):
		""" Callback to get min value from the lidar data coming from the 
		45 degree cone in front of robot

		Args:
			data: ros message of sensor_msgs/LaserScan

		Returns:
			None
		"""
		self.scan_min =  np.min(np.array([data.ranges]))
		print('Value at 0 degrees: ')
		print(data.ranges[0])
		print('Value at 90 degrees: ')
		print(data.ranges[360])
		print('Value at 180 degrees: ')
		print(data.ranges[719])
		
		
	def _lidar_object_check(self):
		""" Use min lidar value in front of robot to check if an object 
			will be hit by moving

		Returns:
			Collision (bool: True if there will be a collision
							 False if no collision will occur 

		"""
		print("Laser lecture: ",self.scan_min)
		
		"""		
		if self.scan_min < 1:
			return True
		else:
			return False
		"""
	
	def listener(self):

		# Init node if script run as main
		if __name__ == '__main__':
			rospy.init_node('area_project', anonymous=True)

		# subscribe to topics
		rospy.Subscriber("/odometry/filtered", Odometry, self._odom_callback)
		rospy.Subscriber("/front/scan", LaserScan, self._front_laser_callback)
	
	def move(self, direction):

		""" Function to move the robot relative to where it is 
			Capable of turning on the spot by 45 degrees cw or ccw
			Capable of going forward or backwards by 1 meter

		Args:
			direction (str): oprions - ("N","NE","E","SE","S","SW","W","NW")

		Returns:
			sucess (int): -1 means failed because move would have caused collision
						   1 means success

		"""
		self.stored_x = self.x_pos
		self.stored_y = self.y_pos

		self.direc = 1

		self.new_twist.angular.z = 0
		self.new_twist.linear.x = 0

		self.target = self.directions[direction]
		self.heading_pub.publish(direction)
		print("Data: ",self.target)
		
		
		# Turn to correct orientation
		print("Pitch: ",self.pitch)
		print("Yaw: ",self.yaw)
		print("Roll: ",self.roll)
		print("x_pos: ",self.x_pos)
		print("y_pos: ",self.y_pos)
		print("z_pos: ",self.z_pos)
		
		
		"""
		while(abs(self.yaw - self.target) > (0.05)):
			self.new_twist.angular.z = 2 * (self.target-self.yaw)
			self.pub.publish(self.new_twist)
		self.new_twist.angular.z = 0
		self.new_twist.linear.x = 100
		self.pub.publish(self.new_twist)
		"""
		self.new_twist.angular.z = 0
		self.pub.publish(self.new_twist)
		self.new_twist.linear.x = 10
		self.pub.publish(self.new_twist)
		if (self._lidar_object_check()):
			# Dont move if there is an obstacle
			return -1
		else:
			return 1
		

		
			
		
if __name__ == '__main__':
	new_movement = Movement()
	new_movement.listener()
	# Ask for imput 
	while not rospy.is_shutdown():
			txt = input("?")
			new_movement.move(txt)
	
