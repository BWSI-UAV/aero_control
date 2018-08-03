#!/usr/bin/env python
# license removed for brevity
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

if __name__ == '__main__':
    # Publishes test images for detector
    try:
        pub = rospy.Publisher('/camera/rgb/image_color', Image, queue_size=10)
        rospy.init_node('image_feeder', anonymous=True)
        rate = rospy.Rate(10)  # 10hz

        cap = cv2.VideoCapture("../images/backpack/test.mp4")
        if not cap.isOpened():
            raise ValueError('cap')

        bridge = CvBridge()
        ctr = 0
        ret, image = cap.read()
        while (ret):
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            pub.publish(bridge.cv2_to_imgmsg(image, "8UC3"))
            ctr += 1
            rospy.loginfo("Image {} posted to {}".format(ctr, pub.name))
            rate.sleep()
            ret, image = cap.read()
    except rospy.ROSInterruptException:
        pass