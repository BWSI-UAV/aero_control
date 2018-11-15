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

#########################
# COORDINATE TRANSFORMS #
#########################
# Create CoordTransforms instance
coord_transforms = coordinate_transforms.CoordTransforms()

##############
# CONTROLLER #
##############
class TranslationController:
    """ Controls drone with purely translaitonal motion, not rotations
    """

    def __init__(self, control_reference_frame='bu'):
        # Create node with name 'controller'
        rospy.init_node('translation_controller')

        self.control_reference_frame=control_reference_frame

        # A subscriber to the topic '/mavros/local_position/pose. self.pos_sub_cb is called when a message of type 'PoseStamped' is recieved 
        self.pos_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pos_sub_cb)
        # Quaternion representing the rotation of the drone's body frame in the NED frame. initiallize to identity quaternion
        self.quat_bu_lenu = (0, 0, 0, 1)

        # A subscriber to the topic '/mavros/state'. self.state_sub_cb is called when a message of type 'State' is recieved
        self.state_sub = rospy.Subscriber("/mavros/state", State, self.state_sub_cb)
        # Flight mode of the drone ('OFFBOARD', 'POSCTL', 'MANUEL', etc.)
        self.mode = State().mode

        # A publisher which will publish the desired linear and anglar velocity to the topic '/setpoint_velocity/cmd_vel_unstamped'
        self.velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size = 1)
        # Linear setpoint velocities
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
        Callback function which is called when a new message of type PoseStamped is recieved by self.position_subscriber.
            Args: 
                - posestamped = ROS PoseStamped message
        """
        self.quat_bu_lenu = (   posestamped.pose.orientation.x, 
                                posestamped.pose.orientation.y, 
                                posestamped.pose.orientation.z, 
                                posestamped.pose.orientation.w)

    def state_sub_cb(self, state):
        """
        Callback function which is called when a new message of type State is recieved by self.state_subscriber.
            
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
        Continually publishes Twist commands in the local lenu reference frame. The values of the 
        Twist command are set in self.vx, self.vy, self.vz and self.wx, self.wy, self.wz.
        """
        # Create twist message for velocity setpoint represented in lenu coords
        velsp__lenu = Twist()

        # Continualy publish velocity commands
        while True:
            # if the stop thread indicator variable is set to True, stop the thread
            if self.stopped:
                return


            # Create velocity setpoint
            # NOTE: velsp__lenu is a Twist message, not a simple array or list. To access and assign the x,y,z
            #       components of the translational velocity, you need to use velsp__lenu.linear.x, 
            #       velsp__lenu.linear.y, velsp__lenu.linear.z
            '''TODO-START: FILL IN CODE HERE 
            Use the provided functions to calculate the desired velocity of the body-up frame with respect to the 
            local ENU frame, expressed in local ENU coordinates (i.e. vsp_bu_lenu__lenu).
            Encode this in the linear portion of the Twist message and assign to the member variable
            self.vel_setpoint_bu_lenu__lenu     
            '''
            raise Exception("CODE INCOMPLETE! Delete this exception and replace with your own code")
            # Set linear velocity (convert command velocity from control_reference_frame to lenu)
            '''TODO-END '''

            # enforce safe velocity limits
            if _MAX_SPEED < 0.0 or _MAX_CLIMB_RATE < 0.0:
                raise Exception("_MAX_SPEED and _MAX_CLIMB_RATE must be positive")
            velsp__lenu.linear.x = min(max(vx,-_MAX_SPEED), _MAX_SPEED)
            velsp__lenu.linear.y = min(max(vy,-_MAX_SPEED), _MAX_SPEED)
            velsp__lenu.linear.z = min(max(vz,-_MAX_CLIMB_RATE), _MAX_CLIMB_RATE)

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
            print('[INFO] Waiting to enter OFFBOARD mode')
            while self.mode != 'OFFBOARD' and not rospy.is_shutdown():
                self.rate.sleep()
            print('[INFO] {} mode ...'.format(self.mode))
    
        # Wait till drone is taken out of OFFBOARD mode
        else:
            print('[INFO] Waiting to exit OFFBOARD mode')
            while self.mode == 'OFFBOARD' and not rospy.is_shutdown():
                self.rate.sleep()
            print('[INFO] {} mode ...'.format(self.mode))

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
            raise ValueError

        dx, dy, dz = displacement
        # Magnitude of dispacement
        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        # time (datetime.timedelta object) is the amount of time the velocity message will be published for
        time = datetime.timedelta(seconds = distance/speed)

        # Record the start time
        start_time = datetime.datetime.now()

        # Set command velocites (vi = di/time)
        self.vx = dx/time.total_seconds()
        self.vy = dy/time.total_seconds()
        self.vz = dz/time.total_seconds()

        print('[INFO] Time of translation: {:.2f}'.format(time.total_seconds()))
        print('[INFO] Displacement vector: {}'.format(displacement))

            # Publish command velocites for time seconds
        while datetime.datetime.now() - start_time < time and not rospy.is_shutdown():
            # print(datetime.datetime.now() - start_time)
            # Note: we don't actually have to call the publish command here. Velocity command are automatically  published 
            # by the stream function which is running in parrellel
            self.rate.sleep()

        # Reset command velocites to 0
        self.vx = self.vy = self.vz = 0
        print('[INFO] Done')


    
if __name__ == "__main__":

    # Create Controller instance
    cframe = 'bu' # Reference frame commands are given in
    controller = TranslationController(control_reference_frame=cframe)
    # Start streaming setpoint velocites
    controller.start()
    controller.wait()

    # Execute maneuver
    # TODO: call controller.translate with a 3-tuple and scalar, positive speed. 
    # 3-tuple is to total change in position desired
    speed = .3
    controller.translate((0.5,0,0), speed)
    controller.translate((0,-0.5,0), speed)
    controller.translate((-0.5,0,0), speed)
    controller.translate((0,0.5,0), speed)

    controller.wait()
    controller.stop()
