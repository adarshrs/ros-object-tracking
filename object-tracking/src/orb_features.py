#!/usr/bin/env python2.7

import numpy as np
import cv2 as cv
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def waterify(image):
    overlay = np.zeros(image.shape, np.uint8)
    overlay[:] = (255, 50, 50)

    water_image = cv.addWeighted(image, 0.4, overlay, 0.1, 0)

    return water_image

def detector(ros_image):
    # Convert ROS image to OpenCV image format
    img = bridge.imgmsg_to_cv2(ros_image, desired_encoding='bgr8')
    img = waterify(img)

    # Initiate ORB detector
    orb = cv.ORB_create(nfeatures=100000, scoreType=cv.ORB_FAST_SCORE)
    
    # find the keypoints with ORB
    kp = orb.detect(img,None)
    
    # compute the descriptors with ORB
    kp, des = orb.compute(img, kp)
    
    # draw only keypoints location,not size and orientation
    img2 = cv.drawKeypoints(img, kp, None, color=(0,255,0), flags=0)
    
    # Convert OpenCV image to ROS image
    feature_image = bridge.cv2_to_imgmsg(img2, encoding="bgr8")

    # Publish on topic "/feature_detector/orb_detection_image"
    pub.publish(feature_image)


if __name__ == '__main__':
    rospy.init_node('feature_detector', anonymous=True)
    rospy.Subscriber('/usb_cam/image_raw', Image, detector)
    pub = rospy.Publisher('/feature_detector/orb_detection_image', Image, queue_size=10)

    bridge = CvBridge()

    rospy.spin()