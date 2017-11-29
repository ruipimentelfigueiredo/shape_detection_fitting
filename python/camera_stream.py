#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


def callback(data):
    bridge = CvBridge()

    try:
      cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # CALL LIBRARY METHOD (COMPUTE BOUNDINGBOXES)
    #bounding_boxes=object.detect(cv_image)

    cv2.imshow("Image window", cv_image)
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
    listener()
