#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Nov 29 03:01:02 2017

@author: atabak
"""
from __future__ import print_function, absolute_import, division
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from shape_detection_fitting import msg

import numpy as np
import os
import sys
import inspect

def callback(data):
    bridge = CvBridge()
    global my_detector, pub

    try:
      cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    rgb_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
    rgb_img_resize = cv2.resize(rgb_img, (300,300))
    boxes, scores, classes = my_detector.detect(rgb_img_resize, threshold=0.5)
    BBA = list() # Bounding box array
    BBA_msg = msg.BoundingBoxArray()
    #BBA_msg.header = Header()
    BBA_msg.header = data.header
    
    for idx in range(len(classes)):
      BB_msg = msg.BoundingBox()
      box = boxes[idx] * 300
      BB_msg.x1 = box[0].astype(np.uint32)
      BB_msg.y1 = box[1].astype(np.uint32)
      BB_msg.x2 = box[2].astype(np.uint32)
      BB_msg.y2 = box[3].astype(np.uint32)
      BB_msg.label = classes[idx].astype(np.uint32)
      BB_msg.confidence = scores[idx].astype(np.float64)
      BBA.append(BB_msg)
      
    BBA_msg.boxes = BBA
    #rospy.loginfo("I heard %d", len(BBA))
    
    pub.publish(BBA_msg)
    
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
  pub = rospy.Publisher('bounding_boxes', msg.BoundingBoxArray)
#  BBA_msg.header = ....
#  BBA_msg.boxes= [boxes]
  
  
  listener()
