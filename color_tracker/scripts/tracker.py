#!/usr/bin/env python

from __future__ import division, print_function

import rospy
import threading
import numpy as np
from geometry_msgs.msg import Twist, Point

import mavros
from mavros_msgs.msg import State

class ColorTracker:
    _YAWRATE = np.pi / 15.0

    def __init__(self, rate=10):
        """ Initializes publishers and subscribers, sets initial values for vars
        :param rate: the rate at which the setpoint_velocity is published
        """
        assert rate > 2 # make sure rate is high enough, if new setpoint recieved within .5 seconds, robot will switch back to POSCTL
        mavros.set_namespace()
        self.state_sub = rospy.Subscriber("/mavros/state", State, self.state_cb)
        self.local_vel_setpnt_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=1)
        self.backpack_pos_sub = rospy.Subscriber("/color_target/pos", Point, self.colortarget_pos_cb)
        self.current_state = None
        self.rate = rospy.Rate(rate)
        self.offboard_point_streaming = False
        self.omega = 0
        while not rospy.is_shutdown() and self.current_state == None:
            pass  # Wait for connection

    def state_cb(self, state):
        """ Starts setpoint streamer when appropriate
        :param state: Given by subscribed topic `/mavros/state`
        """
        self.current_state = state
        mode = getattr(state, "mode", None)
        if (mode is None):
            rospy.logwarn("mode is None") # Mode should be manual by defualt, not None
        elif (mode == "POSCTL"):
            self.start_streaming_offboard_points()
            self.omega = 0
        elif (mode == "MANUAL"):
            self.stop_streaming_offboard_points()

    def colortarget_pos_cb(self, pos):
        """ When target position changes, update desired velocity
        :param pos:
        """
        mode = getattr(self.current_state, "mode", None)
        if (mode is not None and mode == "OFFBOARD"):
            # 0 < x < 480
            self.omega = 0
            if (pos.x < 120):
                self.omega = self._YAWRATE
            elif (pos.x > 360):
                self.omega = -self._YAWRATE

    def start_streaming_offboard_points(self):
        """ Starts thread that will publish yawrate at `rate` in Hz
        """
        def run_streaming():
            self.offboard_point_streaming = True
            while (not rospy.is_shutdown()) and self.offboard_point_streaming:
                vel = Twist()
                vel.angular.z = self.omega
                self.local_vel_setpnt_pub.publish(vel)
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


if __name__ == "__main__":
    rospy.init_node("color_tracker")
    d = ColorTracker()
    rospy.spin()
d.stop_streaming_offboard_points()