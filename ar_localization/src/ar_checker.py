#!/usr/bin/env python
from datetime import datetime
import rospy
import time
import threading
import numpy as np
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from std_msgs.msg import Int32


TARGET_DIS    = 1.0  # target distance away that we want to see the tag
TARGET_THRESH = 0.1  # error/threshold on either side of target_dis that we will allow
TAG_ORDER = [26, 39] # the order you need to visit the tags in.

'''
You need to fly the drone in position control mode to see the AR tags and 
publish their number when you are a (TARGET_DIS +/- TARGET_THRESH) meters away. 
Tag numbers should only be published ONCE per tag seen.

Tags must be visited in the correct order. We are not picky about how you do this:
you can either implement checking in the code so that you do not report seeing
a tag out of order, or you can be very careful how you fly the drone.
'''

class ARDistChecker:
    def __init__(self):
        rospy.loginfo("ARDistChecker Started!")
        '''
        Initializes class: creates subscriber to detect AR tags and publisher to publish which tag it saw.
        
        AR tag data is published on "/ar_pose_marker" with a message of type AlvarMarkers
        
        Which tag you saw should be published on "/seen_tag" as an Int32
        
        TODO: Determine how to initialize a subscriber + publisher for AR tracking.
        Also create any class variables you need to store data (such as which 
        tags you have seen already, etc...)
        '''
        self.ar_pose_sub = None 
        self.ar_tag_seen_pub = None
        raise Exception("Delete this and fill-in subscriber + publisher initialization!")
        
        # Initialize current_marker
        self.current_marker = None
        

    def ar_pose_cb(self,msg):
        '''
        Callback for when the drone sees an AR tag

        Parameters
        ----------
        msg : ar_pose_marker
            list of the poses of ALL the observed AR tags, with respect to the output frame.
            The 1st is not necessarily the closest.
            It can have a length of 0 - that indicates no AR tags were detected.
        
        TODO: figure out what current_marker should be
        '''
        
        raise Exception("Delete this and filter AR messages!")
        self.current_marker = None

        self.check_dist()
        

    def check_dist(self):
        '''
        Finds distance to nearest AR tag and publishes its tag number to "/seen_tag"
        if it is the next tag on our list to see and we haven't seen it before.
        
        Not graded, but nice to have:
        If AR tag is already seen, or is not the next tag to see, will print that to loginfo.
        If new tag is not seen within set distance, tells how far you should go 
        forward to be within range.
        '''
        seen_marker = self.current_marker
        


if __name__ == '__main__':
    rospy.init_node('ar_checker')
    a = ARDistChecker()

    rospy.spin()



