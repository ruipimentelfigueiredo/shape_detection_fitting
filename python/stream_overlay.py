#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Nov 29 03:01:02 2017

@author: atabak
"""
from __future__ import print_function, absolute_import, division
import rospy
from std_msgs.msg import String
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import os
import tensorflow as tf
import sys
import inspect

def callback(data):
    bridge = CvBridge()
    global my_detector

    try:
      cv_image = bridge.imgmsg_to_cv2(data, "rgb8")
    except CvBridgeError as e:
      print(e)

    # CALL LIBRARY METHOD (COMPUTE BOUNDINGBOXES)
    #bounding_boxes=object.detect(cv_image)
    rgb_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
#    cv2.imshow("Image window", cv_image)
    rgb_img_resize = cv2.resize(rgb_img, (300,300))
    my_detector.overlay_shapes(rgb_img_resize)
    cv2.imshow("Image window", rgb_img_resize)
    cv2.waitKey(3)
    
    # TENSORFLOW CODE
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/usb_cam/image_raw", Image, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
  dirname = os.path.dirname(os.path.abspath(inspect.stack()[0][1]))
  root = os.path.dirname(dirname)
  path_to_detector = os.path.join(root, 'lib', 'shape-detection', 'src', 'python')
  sys.path.append(path_to_detector)
  from detector import Detector
  my_detector = Detector()
  listener()
