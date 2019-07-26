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


# Maximum climb rate and speed
_MAX_CLIMB_RATE = 0.5
_MAX_SPEED = 0.5

#Rate to publish velocity commands
_RATE = 10

'''
The drone always proceeds forward at a constant speed. It uses P control 
to automatically regulate its height. The drone's target height is changed from 
the default when it encounters an AR tag obstacle close enough to it.
'''

class ObstacleAvoid:
    def __init__(self):
        '''
        Initializes class. You need to create subscribers to anything you might need
        such as AR tags. Look in your open loop controller and AR tag detectors
        for some sample subscribers
        
        '''
        rospy.init_node('obstacle_avoider')
        
        rospy.loginfo("Obstacle avoidance Started!")
        
        '''
        TODO-START
        Add your subscribers and publishers here!
        The drone can accept both command velocities (which we use) and command positions
        (which we don't use). DO NOT publish both - stick with command velocities.
        '''
        
        raise NotImplementedError("CODE IMCOMPLETE! Replace with your own code")
        
        # Initialize linear setpoint velocities in whatever coordinate frame you choose to use.
        #No vz, as the streaming thread will calculate it from target_height using a proportional controller
        self.vx = None
        self.vy = None
        self.target_height = None
        
        ''' TODO-END '''
        
        # Publishing rate. Do not use this in more than 1 loop at the same time.
        self.rate = rospy.Rate(_RATE)
        
        # Boolean used to indicate if the streaming thread should be stopped
        self.stopped = False

    ######################
    # CALLBACK FUNCTIONS #
    ######################
    
    ''' TODO: Write any needed callback functions here '''

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
        10x second publishes Twist commands in the local lenu reference frame
        and does proportional height control.
        
        Use the similar function from the open loop control lab as a starting point
        
        Uses: 
            - self.vx: x velocity in local coordinates
            - self.vy: y velocity in local coordinates
            - self.target_height: target height (NOT VELOCITY) in local coordinates
        
        Publishes:
            - velocity commands in lenu coordinates
        
        """
        # Create twist message for velocity setpoint represented in lenu coordinates
        velsp__lenu = Twist()

        # This loop runs _RATE times/sec
        while True:
            # if the stop thread indicator variable is set to True, stop the thread
            if self.stopped:
                return
            '''
            TODO-START
            Add proportional controller for height, coordinate frame changes
            '''
            raise NotImplementedError("CODE IMCOMPLETE! Replace with your own code")
            
            ''' TODO-END '''

            # enforce safe velocity limits
            if _MAX_SPEED < 0.0 or _MAX_CLIMB_RATE < 0.0:
                raise Exception("_MAX_SPEED and _MAX_CLIMB_RATE must be positive")
            velsp__lenu.linear.x = min(max(velsp__lenu.linear.x,-_MAX_SPEED), _MAX_SPEED)
            velsp__lenu.linear.y = min(max(velsp__lenu.linear.y,-_MAX_SPEED), _MAX_SPEED)
            velsp__lenu.linear.z = min(max(velsp__lenu.linear.z,-_MAX_CLIMB_RATE), _MAX_CLIMB_RATE)

            # Publish setpoint velocity
            self.velocity_pub.publish(velsp__lenu)

            # Publish velocity at the specified rate
            self.rate.sleep()
    
    def avoidance_routine(self):
        """
        TODO-START
        Runs the avoidance routine. Always proceeds forward at SPEED.
        Stays at the default height until it sees an AR tag close enough to the drone
        then sets the target height to go over or under the tag.
        
        IMPORTANT: the drone can see a marker with a position of (nan nan nan).
        Comparing nan to anything returns false (so nan < 0 is false and nan > 0 is false)
        
        Do not use time.sleep() in this function - use rospy.sleep()
        
        Do some investigation to determine what Alvar thinks is 'up' for AR markers - it is NOT +z
        """
        
        raise NotImplementedError("CODE IMCOMPLETE! Replace with your own code")
        
        ''' TODO-END '''
        rospy.loginfo('Open loop controller: Finished avoidance routine.')

    
if __name__ == "__main__":

    controller = ObstacleAvoid()
    # Start streaming setpoint velocities so we can switch into OFFBOARD mode 
    controller.start()
    
    controller.avoidance_routine()
    
    #Wait for Ctrl-C
    rospy.spin()
    #Stop streaming thread
    controller.stop()
