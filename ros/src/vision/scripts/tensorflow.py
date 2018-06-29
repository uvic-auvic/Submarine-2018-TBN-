#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from scipy.interpolate import UnivariateSpline
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


def create_LUT_8UC1(x, y):
    spl = UnivariateSpline(x, y)
    return spl(xrange(256))

class image_converter:
    def __init__(self):
        # ROS handles
        topic_name = "/video/" + rospy.get_param("~topic_name")
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(topic_name, Image, self.convert)
        self.pub = rospy.Publisher("/video/corrected", Image, queue_size=10)

        # Open CV configutations
        invGamma = 1.0 / rospy.get_param('~gamma')
        self.lookup_table = np.array(
            [((i / 255.0) ** invGamma) * 255 for i in np.arange(0, 256)]).astype("uint8")
        self.incr_ch_lut = create_LUT_8UC1([0, 64, 128, 192, 256], [0, 70, 140, 210, 256])
        self.decr_ch_lut = create_LUT_8UC1([0, 64, 128, 192, 256], [0, 30, 80, 120, 192])
        

    def convert(self, img):
        cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
        rs_image = cv2.resize(cv_image, (600, 400))
        corr_img = self.color_correct(rs_image)
        it_corr_img = self.bridge.cv2_to_imgmsg(corr_img)
        self.pub.publish(it_corr_img, "bgr8")

    def color_correct(self, img):
        b, g, r = cv2.split(img)
        #b = cv2.LUT(b, self.decr_ch_lut).astype(np.uint8)
        g = cv2.LUT(g, self.incr_ch_lut).astype(np.uint8)
        r = cv2.LUT(r, self.incr_ch_lut).astype(np.uint8)
        
        corr_img = cv2.merge((b, g, r))
        #return corr_img
        #return img
        return cv2.LUT(img, self.lookup_table)
        #return cv2.LUT(corr_img, self.lookup_table)

        

if __name__ == '__main__':
    rospy.init_node("tensorflow", anonymous=True)
    ic = image_converter()
    rospy.spin()
    cv2.destroyAllWindows()

