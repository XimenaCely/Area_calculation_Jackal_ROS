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
	right_laser=0
	front_laser=0
	left_laser=0
	
	flag_back=0
	flag_area1=0
	flag_area2=0
	flag_area3=0
	stop=0
	
	area1=0
	area2=0
	area3=0
	areatot=0
	
	
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
		
		#print("x_pos: ",self.x_pos, "y_pos: ",self.y_pos, "z_pos: ",self.z_pos )
	
	def _front_laser_callback(self, data):		
		self.scan_min =  np.min(np.array([data.ranges]))
		self.right_laser=data.ranges[0]
		self.front_laser=data.ranges[360]
		self.left_laser=data.ranges[719]
		print("right_laser: ",self.right_laser, "front_laser: ",self.front_laser, "left_laser: ",self.left_laser )
	
	def listener(self):

		# Init node if script run as main
		if __name__ == '__main__':
			rospy.init_node('area_project', anonymous=True)

		# subscribe to topics
		rospy.Subscriber("/odometry/filtered", Odometry, self._odom_callback)
		rospy.Subscriber("/front/scan", LaserScan, self._front_laser_callback)
	
	def move(self):
		right1=[]
		#Forward without near obstacle
		if self.front_laser > 1 and self.right_laser > 1 and self.left_laser > 1 and self.flag_back==0 and self.flag_area1==0 and self.flag_area2==0 and self.flag_area3==0 and self.stop==0:	
			self.new_twist.angular.z = 0
			self.pub.publish(self.new_twist)
			self.new_twist.linear.x = 10
			self.pub.publish(self.new_twist)
			
		#Obstacle wall in front of the robot
		elif self.front_laser < 1 and self.right_laser > 1 and self.left_laser > 1 and self.flag_back==0 and self.flag_area1==0 and self.flag_area2==0 and self.flag_area3==0 and self.stop==0:	
			self.flag_back=1
			self.new_twist.angular.z = 0
			self.pub.publish(self.new_twist)
			self.new_twist.linear.x = -10
			self.pub.publish(self.new_twist)
			width= self.right_laser + self.left_laser
			self.area1= width*2
			print("area1: ",self.area1)		
			
		elif self.front_laser < 1 and self.right_laser > 1 and self.left_laser > 1 and self.flag_back==1 and self.flag_area1==0 and self.flag_area2==0 and self.flag_area3==0 and self.stop==0:	
			self.new_twist.angular.z = 0
			self.pub.publish(self.new_twist)
			self.new_twist.linear.x = -10
			self.pub.publish(self.new_twist)
			self.flag_area1=1
			
		elif self.front_laser < 1 and self.right_laser > 1 and self.left_laser > 1 and self.flag_back==1 and self.flag_area1==1 and self.flag_area2==0 and self.flag_area3==0 and self.stop==0:	
			self.new_twist.angular.z = 0
			self.pub.publish(self.new_twist)
			self.new_twist.linear.x = -10
			self.pub.publish(self.new_twist)
			
		elif self.front_laser > 1 and self.right_laser > 1 and self.left_laser > 1 and self.flag_back==1 and self.flag_area1==1 and self.flag_area2==0 and self.flag_area3==0 and self.stop==0:	
			self.new_twist.angular.z = 0
			self.pub.publish(self.new_twist)
			self.new_twist.linear.x = -10
			self.pub.publish(self.new_twist)
			self.flag_area2=1
			
			
		elif self.front_laser > 1 and self.right_laser > 1 and self.left_laser > 1 and self.flag_back==1 and self.flag_area1==1 and self.flag_area2==1 and self.flag_area3==0 and self.stop==0:	
			self.new_twist.angular.z = 0
			self.pub.publish(self.new_twist)
			self.new_twist.linear.x = -10
			self.pub.publish(self.new_twist)
			
		elif self.front_laser > 1 and self.right_laser > 1 and self.left_laser > 1 and self.flag_back==1 and self.flag_area1==1 and self.flag_area2==1 and self.flag_area3==1 and self.stop==0:	
			self.new_twist.angular.z = 0
			self.pub.publish(self.new_twist)
			self.new_twist.linear.x = 10
			self.pub.publish(self.new_twist)			
		
		else:
			self.new_twist.angular.z = 0
			self.pub.publish(self.new_twist)
			self.new_twist.linear.x = 0
			self.pub.publish(self.new_twist)
			print("flag_back: ",self.flag_back)
			print("flag_area1: ",self.flag_area1)
			print("flag_area2: ",self.flag_area2)
			print("flag_area3: ",self.flag_area3)
			print("flag_stop: ",self.stop)				
			print("area1: ",self.area1)
			print("area2: ",self.area2)
			print("area3: ",self.area3)
			
			
		if self.front_laser > 16 and self.right_laser > 1 and self.left_laser > 1 and self.flag_back==1 and self.flag_area1==1 and self.flag_area2==1 and self.flag_area3==0 and self.stop==0: 	
			
			self.new_twist.angular.z = 0
			self.pub.publish(self.new_twist)
			self.new_twist.linear.x = 10
			self.pub.publish(self.new_twist)
			width2= self.right_laser + self.left_laser
			self.area2= width2*2
			print("area2: ",self.area2)
			self.flag_area3=1
			
		if self.front_laser > 1 and self.right_laser > 11 and self.left_laser > 1 and self.flag_back==1 and self.flag_area1==1 and self.flag_area2==1 and self.flag_area3==1 and self.stop==0:	
			
			self.new_twist.angular.z = 0
			self.pub.publish(self.new_twist)
			self.new_twist.linear.x = 10
			self.pub.publish(self.new_twist)
			width3= self.right_laser + self.left_laser
			self.area3= width3*2
			print("area3: ",self.area3)
			self.stop=1
			
		if self.front_laser > 1 and self.right_laser > 1 and self.left_laser > 1 and self.flag_back==1 and self.flag_area1==1 and self.flag_area2==1 and self.flag_area3==1 and self.stop==1:
			self.new_twist.angular.z = 0
			self.pub.publish(self.new_twist)
			self.new_twist.linear.x = 0
			self.pub.publish(self.new_twist)
			self.areatot= self.area1 + self.area2 + self.area3
			print("area_total: ",self.areatot)
		
		
if __name__ == '__main__':
	try:
		new_movement = Movement()
		new_movement.listener()
		while not rospy.is_shutdown():
			new_movement.move()
	except KeyboardInterrupt:
		print("End of the program by console")
		
	
