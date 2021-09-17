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
        

def detect(ros_img):

    # Convert ROS image to OpenCV image format
    img = bridge.imgmsg_to_cv2(ros_img, desired_encoding='bgr8')

    # Create mask
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, lower, upper)

    # Filter mask
    if filter:
        mask = cv.medianBlur(mask, window_size)

    # Convert CV2 image to ROS Image
    ros_mask = bridge.cv2_to_imgmsg(mask, encoding="mono8")
    
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
    feature_image = bridge.cv2_to_imgmsg(img, encoding="bgr8")

    # Publish images
    pub1.publish(feature_image)
    pub3.publish(ros_mask)
    # Publish center point
    pub2.publish(ros_point_msg(cX, cY))


if __name__ == '__main__':
    rospy.init_node('feature_detector', anonymous=True)
    
    lower = np.array([rospy.get_param("/color_mask/lower_h"), rospy.get_param("/color_mask/lower_s"), rospy.get_param("/color_mask/lower_v")])
    upper = np.array([rospy.get_param("/color_mask/upper_h"), rospy.get_param("/color_mask/upper_s"), rospy.get_param("/color_mask/upper_v")])
    filter = rospy.get_param("/color_mask/filter")
    if filter:
        window_size = rospy.get_param("/color_mask/window_size")

    lower = lower
    upper = upper
    filter = filter
    window_size = window_size

    bridge = CvBridge()

    rospy.Subscriber('/usb_cam/image_raw', Image, detect)

    pub1 = rospy.Publisher('/feature_detector/detection_image', Image, queue_size=10)
    pub2 = rospy.Publisher('/feature_detector/center_point', Point, queue_size=10)
    pub3 = rospy.Publisher('/mask', Image, queue_size=10)
    
    rospy.spin()