#!/usr/bin/env python

###########
# IMPORTS #
###########
import numpy as np
import rospy
import cv2
import math
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image
from aero_control.msg import Line
import mavros
from mavros_msgs.msg import State
from threading import Thread

import sys, os
sys.path.append(os.path.join(sys.path[0], '../..'))
from common import coordinate_transforms

#############
# CONSTANTS #
#############
_RATE = 10 # (Hz) rate for rospy.rate
_MAX_SPEED = 1 # (m/s)
_MAX_CLIMB_RATE = 0.5 # m/s
_MAX_ROTATION_RATE = 0.5 # rad/s 
IMAGE_HEIGHT = 128
IMAGE_WIDTH = 128
CENTER = np.array([IMAGE_WIDTH//2, IMAGE_HEIGHT//2]) # Center of the image frame. We will treat this as the center of mass of the drone
EXTEND = 64 # Number of pixels forward to extrapolate the line
KP_X = .005 
KP_Y = .005
KP_Z_W = 2
DISPLAY = True

#########################
# COORDINATE TRANSFORMS #
#########################
# Create CoordTransforms instance
coord_transforms = coordinate_transforms.CoordTransforms()

##############
# CONTROLLER #
##############
class LineController:
    '''
    Note: LineController assumes a downward camera reference frame, thus there is
        no configurable input parameter for the reference frame
    '''

    def __init__(self):
        # Create node with name 'tracker'
        rospy.init_node('tracker')

        # A subscriber to the topic '/mavros/local_position/pose. self.pos_sub_cb is called when a message of type 'PoseStamped' is recieved 
        self.pos_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pos_sub_cb)
        # Quaternion representing the rotation of the drone's body frame in the NED frame. initiallize to identity quaternion
        self.quat_bu_lenu = (0, 0, 0, 1)

        # A subscriber to the topic '/mavros/state'. self.state_sub_cb is called when a message of type 'State' is recieved
        self.state_sub = rospy.Subscriber("/mavros/state", State, self.state_sub_cb)
        # State of the drone. 
        #   self.state.mode = flight mode (e.g. 'OFFBOARD', 'POSCTL', etc), 
        #   self.state.armed = are motors armed (True or False), etc.
        self.mode = State().mode

        # A subscriber to the topic '/line/param'. self.line_sub_cb is called when a message of type 'Line' is recieved 
        self.line_sub = rospy.Subscriber('/line/param', Line, self.line_sub_cb)

        # A subscriber to the topic '/aero_downward_camera/image'. self.image_sub_cb is called when a message is recieved 
        self.camera_sub = rospy.Subscriber('/aero_downward_camera/image', Image, self.camera_sub_cb)
        self.image = None

        # A publisher which will publish an image annotated with the detected line to the topic 'line/tracker_image'
        self.tracker_image_pub = rospy.Publisher('/tracker_image', Image, queue_size=1)

        # A publisher which will publish the desired linear and anglar velocity to the topic '/setpoint_velocity/cmd_vel_unstamped'
        self.velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size = 1)

        # Linear setpoint velocities in downward camera frame
        self.vx__dc = 0.0
        self.vy__dc = 0.0
        self.vz__dc = 0.0

        # Yaw setpoint velocities in downward camera frame
        self.wz__dc = 0.0

        # Initialize instance of CvBridge to convert images between OpenCV images and ROS images
        self.bridge = CvBridge()

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

    def camera_sub_cb(self, image):
        """
        Callback function which is called when a new message of type Image is recieved by self.camera_sub.
            Args: 
                - image = ROS Image message
        """
        # Convert Image msg to and OpenCV image
        self.image = cv2.cvtColor(self.bridge.imgmsg_to_cv2(image, "8UC1"), cv2.COLOR_GRAY2BGR)

    def line_sub_cb(self, param):
        """
        Callback function which is called when a new message of type Line is recieved by self.line_sub.
        Notes:
        - This is the function that maps a detected line into a velocity 
        command
            
            Args:
                - param: parameters that define the center and direction of detected line
        """

        '''TODO-START: FILL IN CODE HERE '''
        raise Exception("CODE INCOMPLETE! Delete this exception and replace with your own code")

        # Find the closest point on the line to the center of the image
        # T is the unit vector tangent to the line pointing in the positive x direction 
        # (which is the same as the positive x direction of the bu frame)
        if param.vx >= 0:
            T = np.array([param.vx, param.vy])
        else:
            T = np.array([-param.vx, -param.vy])

        # point is a point on the line


        # closest is the point on the line closest to the center of the image 
        # (https://forum.unity.com/threads/how-do-i-find-the-closest-point-on-a-line.340058/)


        # Aim for a point a distance of EXTEND (in pixels) from the closest point on the line


        # Error between the center of the image and the target point
        

        # Set linear velocities in downward camera frame based on error
        
        

        # Get angle between x-axis and the tangent vector to the line. Calculate direction (+z, -z) using the cross product
        theta = math.acos(np.dot(T, (1, 0))) * np.cross((1, 0, 0), T+[0])[2]
        # Set angular velocites based on error between forward pointing vector and tangent vector
        

        '''TODO-END '''


        ### DO NOT MODIFY ###
    
        # publish tracker commands to an image that can be visualized on
        # a camera feed

        if DISPLAY:
            image = self.image.copy()
            # Draw circle at closest 
            cv2.circle(image, (int(closest[0]), int(closest[1])), 5, (255,128,255), -1)
            # Get unit error vector
            unit_error = error/np.linalg.norm(error)
            # Draw line from CENTER to target
            cv2.line(image, (int(CENTER[0]), int(CENTER[1])), (int(target[0]), int(target[1])), (255, 0, 0), 2)
            # Convert color to a ROS Image message
            image_msg = self.bridge.cv2_to_imgmsg(image, "rgb8")
            # Publish annotated image
            self.tracker_image_pub.publish(image_msg)


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

            # Set linear velocity (convert command velocity from downward camera frame to lenu)
            vx, vy, vz = coord_transforms.get_v__lenu((self.vx__dc, self.vy__dc, self.vz__dc), 
                                                        'dc', self.quat_bu_lenu)
            velsp__lenu.linear.x = vx
            velsp__lenu.linear.y = vy
            velsp__lenu.linear.z = vz

            # Set angular velocity (convert command angular velocity from downward camera to lenu)
            _, _, wz = coord_transforms.get_v__lenu((0.0, 0.0, self.wz__dc), 
                                                    'dc', self.quat_bu_lenu)

            velsp__lenu.angular.x = 0.0
            velsp__lenu.angular.y = 0.0
            velsp__lenu.angular.z = wz

            # enforce safe velocity limits
            if _MAX_SPEED < 0.0 or _MAX_CLIMB_RATE < 0.0 or _MAX_ROTATION_RATE < 0.0:
                raise Exception("_MAX_SPEED,_MAX_CLIMB_RATE, and _MAX_ROTATION_RATE must be positive")
            velsp__lenu.linear.x = min(max(vx,-_MAX_SPEED), _MAX_SPEED)
            velsp__lenu.linear.y = min(max(vy,-_MAX_SPEED), _MAX_SPEED)
            velsp__lenu.linear.z = min(max(vz,-_MAX_CLIMB_RATE), _MAX_CLIMB_RATE)
            velsp__lenu.angular.z = min(max(wz,-_MAX_ROTATION_RATE), _MAX_ROTATION_RATE)

            # Publish setpoint velocity
            self.velocity_pub.publish(velsp__lenu)

            # Publish velocity at the specified rate
            self.rate.sleep()



if __name__ == "__main__":

    # Create Controller instance
    controller = LineController()
    # Start streaming setpoint velocites
    controller.start()

    rospy.spin()

    # Stop streaming setpoint velocites
    controller.stop()
