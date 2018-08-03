#!/usr/bin/env python

from __future__ import division, print_function

import rospy
import threading
import numpy as np
from geometry_msgs.msg import TwistStamped, PoseStamped, Quaternion, Point, Vector3
from sensor_msgs.msg import Image
from aero_control.msg import Line
import cv2
import mavros
from mavros_msgs.msg import State
from cv_bridge import CvBridge, CvBridgeError
from copy import deepcopy

# Variable Notation:
# v__x_y__z: velocity of "x" frame with respect to "y" frame expressed in "z" coordinates
#
# Frame Subscripts:
# m = marker frame (x-right, y-up, z-out when looking at marker)
# dc = downward-facing camera
# fc = forward-facing camera
# bu = body-up frame (x-forward, y-left, z-up, similar to ENU)
# lenu = local East-North-Up world frame ("local" implies that it may not be aligned with east and north, but z is up)

DEBUG = True
NO_ROBOT = False # allows testing on laptop
CONTROL_YAW = True

class LineTracker:
    _IMAGE_HEIGHT = 128
    _IMAGE_WIDTH = 128

    _P_X = 0 # TODO: decide upon initial _P_X
    _P_Y = 0 # TODO: # TODO: decide upon initial K_P_Y
    _P_YAW = 0 # TODO: # TODO: decide upon initial K_P_YAW

    _MAX_SPEED = 0.5 # Only increase with instructor permission

    def __init__(self, rate=10):
        """ Initializes publishers and subscribers, sets initial values for vars
        :param rate: the rate at which the setpoint_velocity is published
        """
        assert rate > 2 # make sure rate is high enough, if new setpoint recieved within .5 seconds, robot will switch back to POSCTL
        mavros.set_namespace()

        # Variables for `self.line_param_cb`
        self.sub_line_param = rospy.Subscriber("/line/param", Line, self.line_param_cb)
        self.bridge = CvBridge()

        # Variables for more debug output
        if DEBUG:
            self.sub_img = rospy.Subscriber("/line/img", Image, self.img_cb)
            self.img = None
            self.pub_alg = rospy.Publisher("/line/alg", Image, queue_size=1)
            self.pub_error = rospy.Publisher("/line/error", Vector3, queue_size=1)

        # Setpoint field expressed as the desired velocity of the body-down frame
        #  with respect to the world frame parameterized in the body-down frame
        self.vsp__bu_lenu__bu = None

        # Variables for `self.state_cb` and start/stop offboard streaming
        self.sub_state = rospy.Subscriber("/mavros/state", State, self.state_cb)
        self.current_state = None
        self.offboard_point_streaming = False
        self.rate = rospy.Rate(rate)
        self.pub_local_velocity_setpoint = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel",
                                                           TwistStamped, queue_size=1)

        while not rospy.is_shutdown() and self.current_state == None:
            pass  # Wait for connection

    def line_param_cb(self, line_params):
        mode = getattr(self.current_state, "mode", None)
        if (mode is not None and mode != "MANUAL") or NO_ROBOT:
            """ Map line paramaterization to a velocity setpoint so the robot will approach and follow the LED strip
            
            Note: Recall the formatting of a Line message when dealing with line_params

            Remember to read the documentation at https://bwsi-uav.github.io/website/line_following.html

            After calculating your various control signals, place them in self.vsp__bu_lenu__bu (which
                is a TwistStamped, meaning self.velocity_setpoint.twist.linear.x is x vel for example)

            Be sure to publish your error using self.pub_error.publish(Vector3(x_error,y_error,0))
            """
            # TODO-START: Calculate error, which should be the x and y components of the
            #   vector from the center of the image to the target
            #   (target should be extrapolated forward along the line ~60 pixels)
            raise Exception("CODE INCOMPLETE! Delete this exception and replace with your own code")
            # TODO-END

            if DEBUG and self.img is not None:
                img_with_drawn_line = self.bridge.imgmsg_to_cv2(self.img, "rgb8")
                # TODO-START: Publish error and an image showing the vectors you use in your calculations
                #   (Publishing image is optional, you should add these vectors to the image defined above)
                raise Exception("CODE INCOMPLETE! Delete this exception and replace with your own code")
                # TODO-END

            # TODO-START: Calculate velocities using p control and put them in `self.vsp__bu_lenu__bu`
            # Remember: Do linear control with only `_P_X` and `_P_Y` first, then test
            # TODO-END

    def state_cb(self, state):
        """ Starts setpoint streamer when mode is "POSCTL" and disables it when mode is "MANUAL"
        :param state: Given by subscribed topic `/mavros/state`
        """
        self.current_state = state
        mode = getattr(state, "mode", None)
        if (mode == "POSCTL" or NO_ROBOT) and not self.offboard_point_streaming:
            rospy.loginfo("Setpoint stream ENABLED")
            self.start_streaming_offboard_points()
        elif mode == "MANUAL" and self.offboard_point_streaming:
            rospy.loginfo("Setpoint stream DISABLED")
            self.stop_streaming_offboard_points()

    def start_streaming_offboard_points(self):
        """ Starts thread that will publish yawrate at `rate` in Hz
        """
        def run_streaming():
            self.offboard_point_streaming = True
            while (not rospy.is_shutdown()) and self.offboard_point_streaming:
                # Publish commands
                if (self.vsp__bu_lenu__bu is not None):
                    # limit speed for safety
                    velocity_setpoint_limited = deepcopy(self.vsp__bu_lenu__bu)
                    speed = np.linalg.norm([velocity_setpoint_limited.twist.linear.x,
                                            velocity_setpoint_limited.twist.linear.y,
                                            velocity_setpoint_limited.twist.linear.z])
                    if speed > self._MAX_SPEED:
                        velocity_setpoint_limited.twist.linear.x *= self._MAX_SPEED / speed
                        velocity_setpoint_limited.twist.linear.y *= self._MAX_SPEED / speed
                        velocity_setpoint_limited.twist.linear.z *= self._MAX_SPEED / speed

                    # Publish limited setpoint
                    self.pub_local_velocity_setpoint.publish(velocity_setpoint_limited)
                self.rate.sleep()

        self.offboard_point_streaming_thread = threading.Thread(target=run_streaming)
        self.offboard_point_streaming_thread.start()

    def stop_streaming_offboard_points(self):
        """ Safely terminates offboard publisher
        """
        self.offboard_point_streaming = False
        try:
            self.offboard_point_streaming_thread.join()
        except AttributeError:
            pass

    def img_cb(self, msg):
        """ Updates debug img
        """
        self.img = msg

if __name__ == "__main__":
    rospy.init_node("line_tracker")
    d = LineTracker()
    rospy.spin()
d.stop_streaming_offboard_points()
