#!/usr/bin/env python

###########
# IMPORTS #
###########
import numpy as np
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from aero_control.msg import Line
import sys

#############
# CONSTANTS #
#############
LOW = None# Lower image thresholding bound
HI = None # Upper image thresholding bound
LENGTH_THRESH = None # If the length of the largest countour is less than LENGTH_THRESH, we will not consider it a line
KERNEL  = np.ones((5,5),np.uint8)
DISPLAY = True

class LineDetector:
    def __init__(self):
        # Create node with name 'detector'
        rospy.init_node('detector') 

        # A subscriber to the topic '/aero_downward_camera/image'. self.image_sub_cb is called when a message is recieved 
        self.camera_sub = rospy.Subscriber('/aero_downward_camera/image', Image, self.camera_sub_cb)

        # A publisher which will publish a parametrization of the detected line to the topic '/line/param'
        self.param_pub = rospy.Publisher('/line/param', Line, queue_size=1)

        # A publisher which will publish an image annotated with the detected line to the topic 'line/detector_image'
        self.detector_image_pub = rospy.Publisher('/line/detector_image', Image, queue_size=1)

        # Initialize instance of CvBridge to convert images between OpenCV images and ROS images
        self.bridge = CvBridge()


    ######################
    # CALLBACK FUNCTIONS #
    ######################
    def camera_sub_cb(self, image):
        """
        Callback function which is called when a new message of type Image is recieved by self.camera_sub.
            Args: 
                - msg = ROS Image message
        """
        # Convert Image msg to and OpenCV image
        image = self.bridge.imgmsg_to_cv2(image, "8UC1")

        # Detect line in the image. detect returns a  parameterize the line (if one exists)
        line = self.detect_line(image)

        # If a line was detected, publish the parameterization to the topic '/line/param'
        if line is not None:
            msg = Line()
            msg.x, msg.y, msg.vx, msg.vy = line
            # Publish param msg
            self.param_pub.publish(msg)

    ##########
    # DETECT #
    ##########
    def detect_line(self, image):
        """ 
        Given an image, fit a line to biggest contour if it meets size requirements (otherwise return None)
        and return a paramaterization of the line as a center point on the line and a vector
        pointing in the direction of the line.
            Args:
                - image = OpenCV image
            Returns: (x, y, vx, vy) where (x, y) is the centerpoint of the line in image and 
            (vx, vy) is a vector pointing in the direction of the line. Both values are given
            in downward camera pixel coordinates. Returns None if no line is found
        """

        '''TODO-START: FILL IN CODE HERE '''
        raise Exception("CODE INCOMPLETE! Delete this exception and replace with your own code")

        # Get a colored copy of the image. This will be used solely to provide an annotated version
        # of the image for debuging purposes
        

        # Threshold the image and find contours in the masked image
        # If contours exist, we will consider the countour with the most area to be our line
        
            # Fit a rectangle of smallest area possible around the max countour and get the hight and width of the rectangle. 
            # If neither value in longer than LENGTH_THRESH, we
            # will not consider the countour to be a line
            

            # Fit a line to the max countour. This will be our line to return. fitLine() return values are length 1 numpy arrays
            
            # Publish a copy of the image annotated with the detected line (only if display is true)
            
                # Draw the rectangle around the countour. Hint: Use cv2.boxPoints() and cv2.drawContours()
                

                # Draw point at (x, y). Hint: use cv2.circle()
                

                # Draw vector parallel to the fitted line at point (x,y)). 
                # Hint: just use cv2.line() and multiply (vx,vy) by some scalar to make the line 
                # a certain length and find the second point to draw the line
                
                # Convert color image to a ROS Image message
                
                # Publish annotated image
                

                return x, y, vx, vy


        # Publish image even if no line is detected
        
            # Convert color to a ROS Image message
            
            # Publish annotated image with no contours or lines overlayed
            

        # If no countors were found, return None
        return None
        '''TODO-END '''


if __name__ == '__main__':
    # Initialize detector
    detector = LineDetector()
    rospy.loginfo("Line detector initialized")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    except Exception as e:
        print(e)
