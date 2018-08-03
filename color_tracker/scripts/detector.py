#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import sys

# Preset thresholds for red
rgb_low = np.array([120, 0, 0])
rgb_high = np.array([255, 100, 100])
DEBUG = True

class ColorDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_color", Image, self.image_cb)
        self.position_pub = rospy.Publisher("/color_target/pos", Point, queue_size=3)
        self.image_pub = rospy.Publisher("/color_target/img", Image, queue_size=1)

    def image_cb(self, data):
        """ Find center of largest blob fitting color threshold
        :param data: Image data given by publisher
        """
        image = self.bridge.imgmsg_to_cv2(data, "8UC3")
        mask = cv2.inRange(image, rgb_low, rgb_high)
        cv2.morphologyEx(image, cv2.MORPH_OPEN, np.ones((8, 8), np.uint8), mask)

        _, contours, _ = cv2.findContours(mask, mode=cv2.RETR_LIST, method=cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0:
            maxc = max(contours, key=cv2.contourArea)
            M = cv2.moments(maxc)
            if (M["m00"] != 0):
                msg = Point()
                msg.x = int(M["m10"] / M["m00"]) # Finds center
                msg.y = int(M["m01"] / M["m00"])
                msg.z = -1
                self.position_pub.publish(msg)
                # Create image with detection mask
                if (DEBUG):
                    # Highlight thresholded areas
                    maskImg = np.zeros(image.shape, image.dtype)
                    maskImg[:, :] = (255, 0, 255)
                    maskViz = cv2.bitwise_and(maskImg, maskImg, mask=mask)
                    cv2.addWeighted(maskViz, 1, image, 0.6, 0, image)
                    # Box main contour
                    box = cv2.minAreaRect(maxc)
                    box = cv2.boxPoints(box)
                    box = np.int0(box)
                    cv2.drawContours(image, [box], 0, (0, 255, 0), 2)
                    # Draw center
                    cv2.circle(image, (msg.x, msg.y), 3, (255, 255, 255), -1)
                    self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, "rgb8"))

if __name__ == '__main__':
    # Use alternate thresholds if given
    rospy.init_node('color_detector', anonymous=True)
    if (len(sys.argv) == 7):
        rgb_low = np.array([int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3])])
        rgb_high = np.array([int(sys.argv[4]), int(sys.argv[5]), int(sys.argv[6])])
        rospy.loginfo("Using Given Threshs: {} - {}".format(rgb_low, rgb_high))
    else:
        rospy.loginfo("Using Default Threshs: {} - {}".format(rgb_low, rgb_high))
    # Start Node
    bd = ColorDetector()
    rospy.loginfo("Color detector initialized")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    except Exception as e:
        print(e)