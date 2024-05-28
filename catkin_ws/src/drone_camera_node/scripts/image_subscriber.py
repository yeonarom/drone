#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

def image_callback(msg):
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow("Camera Image", cv_image)
        cv2.waitKey(1)
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

def main():
    rospy.init_node('image_subscriber', anonymous=True)
    rospy.Subscriber("/my_drone/image_raw", Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
