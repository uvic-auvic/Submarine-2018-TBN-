#!/usr/bin/env python

import rospy
import cv2
import sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:
    def __init__(self, img_topic):
        self.sub = rospy.Subscriber(img_topic, Image, self.convert)
        self.bridge = CvBridge()

    def convert(self, img):
        cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
        cv2.imshow("Python Image Transport", cv_image)
        cv2.waitKey(3)

if __name__ == '__main__':
    rospy.init_node("tensorflow", anonymous=True)
    ic = image_converter("/video/" + "top_face_cam")
    
    

