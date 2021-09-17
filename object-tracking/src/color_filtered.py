#!/usr/bin/env python2.7

import numpy as np
import cv2 as cv
import rospy
import message_filters
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge


def ros_point_msg(x, y, z=0):
    msg = Point()
    msg.x = x
    msg.y = y
    msg.z = z

    return msg
    

class objectTracker:
    def __init__(self):
        self.ros_mask = None
        self.ros_img = None

        self.bridge = CvBridge()

        rospy.Subscriber('/mask', Image, self.get_mask)
        rospy.Subscriber('/usb_cam/image_raw', Image, self.get_image)

        self.pub1 = rospy.Publisher('/feature_detector/detection_image', Image, queue_size=10)
        self.pub2 = rospy.Publisher('/feature_detector/center_point', Point, queue_size=10)


    def get_mask(self, mask):
        self.ros_mask = mask
        self.detect()
    
    def get_image(self, img):
        self.ros_img = img
        self.detect()

    def detect(self):
        if self.ros_mask is None or self.ros_img is None:
            return

        # Convert ROS image to OpenCV image format
        mask = self.bridge.imgmsg_to_cv2(self.ros_mask, desired_encoding='mono8')
        img = self.bridge.imgmsg_to_cv2(self.ros_img, desired_encoding='bgr8')
        
        # Find and plot contours
        im, contours, hierarchy = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
        cv.drawContours(image=img, contours=contours, contourIdx=-1, color=(0, 255, 0), thickness=2, lineType=cv.LINE_AA)
        
        # Find center of mask (using center of mass equation)
        x_nums = np.reshape(np.arange(0, mask.shape[1]), (mask.shape[1], 1))
        y_nums = np.reshape(np.arange(0, mask.shape[0]), (mask.shape[0], 1))
        cX = sum(np.dot(mask, x_nums))/np.sum(mask)
        cY = sum(np.dot(y_nums.T, mask).T)/np.sum(mask)

        # Plot center of mask on image
        img = cv.circle(img, (cX,cY), radius=4, color=(0, 0, 255), thickness=6)

        # Convert OpenCV image to ROS image
        feature_image = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")

        # Publish images
        self.pub1.publish(feature_image)
        # Publish center point
        self.pub2.publish(ros_point_msg(cX, cY))


if __name__ == '__main__':
    rospy.init_node('feature_detector', anonymous=True)
    objectTracker()
    rospy.spin()