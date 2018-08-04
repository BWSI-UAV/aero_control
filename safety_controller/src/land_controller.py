#!/usr/bin/env python
import rospy
import mavros

from geometry_msgs.msg import Twist, PoseStamped, TwistStamped, PoseArray
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandTOL
import numpy as np

'''
Authored by Caleb Trotz, August 2018
This node lands Intel Aero drones in offboard if they are above
a certain height, or if their velocity commands exceed
a certain threshold in magnitude.
'''

_MAX_HEIGHT = 1.0 # meters
_MAX_VEL = 0.5 # m/s

_MAX_Z_VEL = 0.1 # m/s

class LandController:
	def __init__(self):
		mavros.set_namespace()

		rospy.Subscriber("/mavros/state", State, self.state_cb)
		rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_cb)
		rospy.Subscriber("/mavros/setpoint_velocity/cmd_vel", TwistStamped, self.vel_cb)

		self.state = None
		rospy.wait_for_service('/mavros/cmd/land')
		self.land_srv = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)


	def state_cb(self,msg):
		self.state = msg

	def pose_cb(self,msg):
		height = msg.pose.position.z

		if height > _MAX_HEIGHT and self.state is not None and self.state.mode == "OFFBOARD": 
			rospy.logerr("landing. too high!")
			self.land_srv(0,0,0,0,0)

	def vel_cb(self,msg):
		linear_vel = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]

		vel_mag = np.linalg.norm(linear_vel)

		if vel_mag > _MAX_VEL and self.state is not None and self.state.mode == "OFFBOARD":
			rospy.logerr("landing. too much total vel!")
			self.land_srv(0,0,0,0,0)
		if linear_vel[2] > _MAX_Z_VEL and self.state is not None and self.state.mode == "OFFBOARD":
			rospy.logerr("landing. too much z vel")
			self.land_srv(0,0,0,0,0)

if __name__== "__main__":
	rospy.init_node("land_controller")

	a = LandController()

	rospy.spin()
