#!/usr/bin/env python
from time import time
import rospy
import time
from threading import Thread
import numpy as np
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker

from geometry_msgs.msg import Twist, PoseStamped
import mavros
from mavros_msgs.msg import State

import sys, os
sys.path.append(os.path.join(sys.path[0], '../..'))
from common import coordinate_transforms

# Create CoordTransforms instance
coord_transforms = coordinate_transforms.CoordTransforms()


#Distance from tag to start avoidance routine
TARGET_DIS = 1.0
AVOID_LENGTH = 1.5

#Default height to fly at (and threshold height for determining whether to fly over or under AR tag)
DEFAULT_HEIGHT = 0.7
#Change in height when we see the AR tag
DELTA_HEIGHT = 0.5

# To control target height
P_Z = 1.5
# Forward speed
SPEED = 0.3
_MAX_CLIMB_RATE = 1
_MAX_SPEED = 0.8

#Rate to publish velocity commands
_RATE = 10

'''
The drone should to proceed forward at a constant speed. It uses P control 
to automatically regulate its height. The drone's target height is changed from 
the default when it encounters an AR tag obstacle close enough to it.
'''

class ObstacleAvoid:
    def __init__(self):
        '''
        Initializes class.
        
        '''
        rospy.init_node('obstacle_avoider')
        
        rospy.loginfo("Obstacle avoidance Started!")
        
        # A subscriber to the topic '/mavros/local_position/pose. self.pos_sub_cb is called when a message of type 'PoseStamped' is recieved 
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pos_sub_cb)
        # Quaternion representing the rotation of the drone's body frame (bu) in the LENU frame. 
        # Initialize to identity quaternion, as bu is aligned with lenu when the drone starts up.
        self.quat_bu_lenu = (0, 0, 0, 1)
        
        #Keeps track of current height
        self.height = 0

        # Subscribe to AR data from ALvar
        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.ar_pose_cb) 
        
        #reference frame for commands
        self.control_reference_frame='bu'
        
        # A publisher to publish the desired linear and angular velocity to the topic '/setpoint_velocity/cmd_vel_unstamped'
        self.velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size = 1)
        
        # Initialize linear setpoint velocities. No vz, as the streaming thread 
        # will calculate it from target_height using a proportional controller
        self.vx = SPEED
        self.vy = 0
        self.target_height = DEFAULT_HEIGHT
        
        # Initialize current marker. This is the last marker we saw
        self.marker = None
        
        # Publishing rate. Do not use this in more than 1 loop at the same time.
        self.rate = rospy.Rate(_RATE)
        
        # Boolean used to indicate if the streaming thread should be stopped
        self.stopped = False

    ######################
    # CALLBACK FUNCTIONS #
    ######################
    def pos_sub_cb(self, posestamped):
        """
        Updates the orientation the drone (the bu frame) related to the lenu frame
        and the drone's current height.
            Args: 
                - posestamped = ROS PoseStamped message
        """
        self.quat_bu_lenu = (posestamped.pose.orientation.x, 
                             posestamped.pose.orientation.y, 
                             posestamped.pose.orientation.z, 
                             posestamped.pose.orientation.w)
        self.height = posestamped.pose.position.z

    def state_sub_cb(self, state):
        """
        Callback function which is called when a new message of type State is recieved by self.state_subscriber to update the drone's mode (MANUAL, POSCTL, or OFFBOARD)
            
            Args:
                - state = mavros State message
        """
        self.mode = state.mode
    
    def ar_pose_cb(self,msg):
        '''Callback for when the drone sees an AR tag

        Parameters
        ----------
        msg : ar_pose_marker
            list of the poses of ALL the observed AR tags, with respect to the output frame.
            The 1st is not necessarily the closest.
            It can have a length of 0 - that indicates no AR tags were detected.
        
        '''
        if len(msg.markers) < 1: 
            return

        self.marker = min(msg.markers, key=lambda p: p.pose.pose.position.z)


    #############
    # STREAMING #
    #############
    def start(self): 
        """
        Start thread to stream velocity commands.
        """
        self.offboard_command_streaming_thread = Thread(target=self.stream_offboard_velocity_setpoints)
        self.offboard_command_streaming_thread.start()

    def stop(self):
        """
        Stop streaming thread.
        """
        self.stopped = True
        try:
            self.offboard_command_streaming_thread.join()
        except AttributeError:
            pass

    def stream_offboard_velocity_setpoints(self):
        """
        Continually publishes Twist commands in the local lenu reference frame.
        Our desired velocities (in the local frame) are in self.vx, self.vy, self.vz (linear velocity) 
        and self.wx, self.wy, self.wz (rotational velocities around their respective axes)
        
        Also does proportional height control.
        """
        # Create twist message for velocity setpoint represented in lenu coordinates
        velsp__lenu = Twist()

        # Continually publish velocity commands
        while True:
            # if the stop thread indicator variable is set to True, stop the thread
            if self.stopped:
                return

            # Calculate our desired vertical velocity in the bu frame from height setpoint
            vz__bu = (self.target_height - self.height) * P_Z
            # Create velocity setpoint
            # NOTE: velsp__lenu is a Twist message, not a simple array or list. To access and assign the x,y,z
            #       components of the translational velocity, you need to use velsp__lenu.linear.x, 
            #       velsp__lenu.linear.y, velsp__lenu.linear.z 
            vx, vy, vz = coord_transforms.get_v__lenu((self.vx, self.vy, vz__bu), 
                                                    self.control_reference_frame, 
                                                    self.quat_bu_lenu)
            velsp__lenu.linear.x = vx
            velsp__lenu.linear.y = vy
            velsp__lenu.linear.z = vz

            # enforce safe velocity limits
            if _MAX_SPEED < 0.0 or _MAX_CLIMB_RATE < 0.0:
                raise Exception("_MAX_SPEED and _MAX_CLIMB_RATE must be positive")
            velsp__lenu.linear.x = min(max(velsp__lenu.linear.x,-_MAX_SPEED), _MAX_SPEED)
            velsp__lenu.linear.y = min(max(velsp__lenu.linear.y,-_MAX_SPEED), _MAX_SPEED)
            velsp__lenu.linear.z = min(max(velsp__lenu.linear.z,-_MAX_CLIMB_RATE), _MAX_CLIMB_RATE)
            
            '''TODO-END '''

            # Publish setpoint velocity
            self.velocity_pub.publish(velsp__lenu)

            # Publish velocity at the specified rate
            self.rate.sleep()
    
    def avoidance_routine(self):
        """
        Runs the avoidance routine. Always proceeds forward at SPEED.
        Stays at the default height until it sees an AR tag close enough when the
        drone is in offboard mode, then sets the target height
        to go over or under the tag.
        
        IMPORTANT: the drone can see a marker with a position of (nan nan nan).
        Comparing nan to anything returns false (so nan < 0 is false and nan > 0 is false)
        
        Do not use time.sleep() - use rospy.sleep()
        
        Do some investigation to determine what Alvar thinks is 'up' for AR markers - it is NOT +z
        """
        self.vx = SPEED
        self.vy = 0
        self.target_height = DEFAULT_HEIGHT       
        
        # Wait until marker is in range.
        while self.marker == None or not self.marker.pose.pose.position.z < TARGET_DIS:
            rospy.sleep(1/_RATE)
        
        rospy.loginfo('Open loop controller: Saw marker {} m away. Starting avoidance routine.'.format(self.marker.pose.pose.position.z))
        
        marker_height = self.height - self.marker.pose.pose.position.y
        rospy.loginfo('Saw marker at {}'.format(marker_height))
        
        if marker_height < DEFAULT_HEIGHT:
            self.target_height = DEFAULT_HEIGHT + DELTA_HEIGHT
        else:
            self.target_height = DEFAULT_HEIGHT - DELTA_HEIGHT
        
        #Wait for the drone to finish the avoidance maneuver then return to default height
        rospy.sleep(AVOID_LENGTH/SPEED)
        self.target_height = DEFAULT_HEIGHT
            
        rospy.loginfo('Open loop controller: Finished avoidance routine.')

    
if __name__ == "__main__":

    controller = ObstacleAvoid()
    # Start streaming setpoint velocities so we can switch into OFFBOARD mode 
    controller.start()
    controller.avoidance_routine()
    rospy.spin()
    controller.stop()
