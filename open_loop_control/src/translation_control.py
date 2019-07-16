#!/usr/bin/env python

###########
# IMPORTS #
###########
import numpy as np
import rospy
from geometry_msgs.msg import Twist, PoseStamped
import mavros
from mavros_msgs.msg import State
from threading import Thread
import math
import datetime

import sys, os
sys.path.append(os.path.join(sys.path[0], '../..'))
from common import coordinate_transforms

#############
# CONSTANTS #
#############
_RATE = 10 # (Hz) rate for rospy.rate
_MAX_SPEED = 1 # (m/s)
_MAX_CLIMB_RATE = 0.5 # m/s 
_COORDINATE_FRAMES = {'lenu','lned','bu','bd','dc','fc'}

#########################
# COORDINATE TRANSFORMS #
#########################
# Create CoordTransforms instance
coord_transforms = coordinate_transforms.CoordTransforms()

##############
# CONTROLLER #
##############
class TranslationController:
    """ 
    Controls drone with open-loop translational motion, not rotations.
    
    This only works if the drone has a good estimate of its position:
    if qGroundControl says: FAILSAFE ENABLED: no local position, you need to 
    restart optical flow and teraranger, else this code will not work.
    """

    def __init__(self, control_reference_frame='bu'):
        # Create node with name 'controller'
        rospy.init_node('translation_controller')
        
        if control_reference_frame not in _COORDINATE_FRAMES:
            raise ValueError("Invalid control reference frame: " + control_reference_frame)

        self.control_reference_frame=control_reference_frame

        # A subscriber to the topic '/mavros/local_position/pose. self.pos_sub_cb is called when a message of type 'PoseStamped' is recieved 
        self.pos_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pos_sub_cb)
        # Quaternion representing the rotation of the drone's body frame (bu) in the LENU frame. 
        # Initiallize to identity quaternion, as bu is aligned with lenu when the drone starts up.
        self.quat_bu_lenu = (0, 0, 0, 1)

        # A subscriber to the topic '/mavros/state'. self.state_sub_cb is called when a message of type 'State' is recieved
        self.state_sub = rospy.Subscriber("/mavros/state", State, self.state_sub_cb)
        # Flight mode of the drone ('OFFBOARD', 'POSCTL', 'MANUAL', etc.)
        self.mode = State().mode

        # A publisher which will publish the desired linear and anglar velocity to the topic '/setpoint_velocity/cmd_vel_unstamped'
        self.velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size = 1)
        # Initialize linear setpoint velocities
        self.vx = 0
        self.vy = 0
        self.vz = 0

        # Publishing rate
        self.rate = rospy.Rate(_RATE)

        # Boolean used to indicate if the streaming thread should be stopped
        self.stopped = False


    ######################
    # CALLBACK FUNCTIONS #
    ######################
    def pos_sub_cb(self, posestamped):
        """
        Updates the orientation the drone (the bu frame) related to the lenu frame
            Args: 
                - posestamped = ROS PoseStamped message
        """
        self.quat_bu_lenu = (   posestamped.pose.orientation.x, 
                                posestamped.pose.orientation.y, 
                                posestamped.pose.orientation.z, 
                                posestamped.pose.orientation.w)

    def state_sub_cb(self, state):
        """
        Callback function which is called when a new message of type State is recieved 
        by self.state_subscriber to update the drone's mode (MANUAL, POSCTL, or OFFBOARD)
            
            Args:
                - state = mavros State message
        """
        self.mode = state.mode


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
        Continually publishes Twist commands in the local lenu reference frame. Our desired velocities (in the local frame) are in self.vx, self.vy, self.vz (linear velocity) 
        and self.wx, self.wy, self.wz (rotational velocities around their respective axes)
        """
        # Create twist message for velocity setpoint represented in lenu coordinates
        velsp__lenu = Twist()

        # Continually publish velocity commands
        while True:
            # if the stop thread indicator variable is set to True, stop the thread
            if self.stopped:
                return

            '''
            Create velocity setpoint
            NOTE: velsp__lenu is a Twist message, not a simple array or list. It
            is composed of two 3-long vectors: linear and angular.
            To access and assign the x,y,z components of the translational velocity, 
            you need to use velsp__lenu.linear.x, etc.
            See documentation here: https://docs.ros.org/melodic/api/geometry_msgs/html/msg/Twist.html
            
            TODO-START: FILL IN CODE HERE 
            Use the provided functions to calculate the desired velocity of the body-up frame with respect to the 
            local ENU frame, expressed in local ENU coordinates (i.e. vsp_bu_lenu__lenu).
            Encode this in the linear portion of the Twist message.
            
            Use the coord_transforms.get_v__lenu function in aero_control/common/coordinate_transforms.py 
            to convert velocities in control_reference_frame to lenu frame.
            
            Don't forget to prevent vx, vy from exceeding _MAX_SPEED and vz from exceeding _MAX_CLIMB_RATE
            '''
            raise Exception("CODE INCOMPLETE! Delete this exception and replace with your own code")
            
            '''TODO-END '''

            # Publish setpoint velocity
            self.velocity_pub.publish(velsp__lenu)

            # Publish velocity at the specified rate
            self.rate.sleep()

    ########
    # WAIT #
    ########
    def wait(self):
        """
        If the drone is not in OFFBOARD mode, loops until the drone is put into OFFBOARD mode. If the drone is in OFFBOARD mode,
        loops until the drone is taken out of OFFBOARD mode.
        """
        # Wait till drone is put into OFFBOARD mode
        if self.mode != 'OFFBOARD':
            rospy.loginfo('Open loop controller: Waiting to enter OFFBOARD mode')
            while self.mode != 'OFFBOARD' and not rospy.is_shutdown():
                self.rate.sleep()
            rospy.loginfo('Open loop controller: {} mode ...'.format(self.mode))
    
        # Wait till drone is taken out of OFFBOARD mode
        else:
            rospy.loginfo('Open loop controller: Waiting to exit OFFBOARD mode')
            while self.mode == 'OFFBOARD' and not rospy.is_shutdown():
                self.rate.sleep()
            rospy.loginfo('Open loop controller: {} mode ...'.format(self.mode))

    ###############
    # TRANSLATION #
    ###############
    def translate(self, displacement, speed):
        """
        Given a displacement vector (dx, dy, dz) and a speed, sets the command velocities so that the drone moves from its current
        location (x, y, z) to the point (x+dx, y+dy, z+dz),
            Args:
                - displacement = (dx, dy, dz) (m)
                - speed = (m/s), mush be positive
        """
        # Clip speed at _MAX_SPEED
        speed = min(speed, _MAX_SPEED)
        # Raise error if speed is 0 or not positive
        if speed <= 0:
            raise ValueError("Speed must be positive")
        ''' TODO-START
        Set self.vx, self.vy, self.vz to the correct speeds, and set the correct time 
        for the velocity commands to be published for.
        '''
        raise Exception("CODE INCOMPLETE! Delete this exception and replace with your own code")
        move_time = 0
        
        ''' TODO-END'''
        
        rospy.loginfo('Open loop controller: Time of translation: {:.2f}'.format(move_time))
        rospy.loginfo('Open loop controller: Displacement vector: {}'.format(displacement))
        
        # Wait for us to finish publishing velocities
        rospy.sleep(move_time)

        # Reset command velocites to 0
        self.vx = self.vy = self.vz = 0
        rospy.loginfo('Open loop controller: Done')


    
if __name__ == "__main__":

    # Create Controller instance
    cframe = 'bu' # Reference frame commands are given in
    controller = TranslationController(control_reference_frame=cframe)
    # Start streaming setpoint velocites
    controller.start()
    controller.wait()
    
    '''
    Execute maneuver
    TODO-START: call controller.translate with a 3-tuple and scalar, positive speed. 
    3-tuple is to total change in position desired
    '''
    raise Exception("CODE INCOMPLETE! Delete this exception and replace with your own code")
    ''' TODO-END '''

    controller.wait()
    controller.stop()
