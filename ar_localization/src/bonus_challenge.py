#!/usr/bin/env python
from datetime import datetime
import rospy
import time
import threading
import numpy as np
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Int32


TARGET_DIS    = 1.0  # target distance away that we want to see the tag
TARGET_HEIGHT    = 0.6  # height the student should fly at
HEIGHT_ERROR    = .1 #allowable flight height error 

'''
You need to fly the drone in position control mode to see the AR tag and stay in the green zone for height
(TARGET_HEIGHT-HEIGHT_ERROR<height<TARGET_HEIGHT+HEIGHT_ERROR)
once it sees the AR tag and is <= TARGET_DIS away from it, it should get the height of the tag and 
if the tag is above the TARGET_HEIGHT then tell pilot to go under
if it is below, then tell pilot to go over
'''

class BasicAvoider:
    def __init__(self):
        '''
        Initializes class: creates subscriber to detect AR tags and height
        '''

        rospy.loginfo("BasicAvoider Started!")

        '''
        Initializes class: creates subscriber to detect AR tags and subscriber to drone pose.
        -AR tag data is published on "/ar_pose_marker" with a message of type AlvarMarkers
        -Pose data is published on '/mavros/local_position/pose' with message type PoseStamped
        TODO: Determine how to initialize a subscriber for AR tracking and pose.
        '''

        self.ar_pose_sub = None 
        self.pos_sub = None
        raise Exception("Delete this and fill in subscriber nitialization!")

    ######################
    # CALLBACK FUNCTIONS #
    ######################

    def ar_pose_cb(self,msg):
        '''
        Callback for when the drone sees an AR tag
        Parameters
        ----------
        msg : ar_pose_marker
            msg.markers = list of ALL the observed AR tags, with respect to the output frame.
            The 1st is not necessarily the closest.
            It can have a length of 0 - that indicates no AR tags were detected.
        
        TODO: Find closest marker or simply return if no markers. Once you have found the marker, 
        check if it is within TARGET_DIS, if it is,
        find its height from the ground (remember, not just the height relative to the drone!),
        then print to the console for the pilot to go over or under based on that height
        -marker.pose.pose.position.z is ar's distance in front of drone
        -marker.pose.pose.position.y is ar's height relative to drone (down is possitive, up is negative)
        -marker.pose.pose.position.x is ar's distance from side of drone (pos is right)
        '''
        rospy.loginfo("This is how to print on console")
        raise Exception("Delete this and fill in your ar code")

    def pos_sub_cb(self, posestamped):
        '''
        Callback for when the drone recieves pose info
        Parameters
        ----------
        posestamped : PoseStamped
            drone's possition information
        
        TODO: Set self.height equal to the drone's height and then use that height to determine if the drone
        is flying withing the green zone. If it is flying in the green zone print nothing or a good job message,
        if it is not flying within the green zone, print a warning message telling the pilot
        what height the drone is at now and what height it should be at
        -posestamped.pose.position.z = drone height
        '''
        raise Exception("Delete this and fill in your ar code")

if __name__ == '__main__':
    rospy.init_node('final_challenge_sol')
    ba = BasicAvoider()

    rospy.spin()
