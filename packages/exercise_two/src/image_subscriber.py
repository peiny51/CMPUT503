#!/usr/bin/env python3

import os
import numpy as np
import cv2
from cv_bridge import CvBridge
from collections import deque

import rospy

from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage


class ImageSubscriber(DTROS):
    def __init__(self, node_name):
        super(ImageSubscriber, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.sub = rospy.Subscriber("/"+os.environ['VEHICLE_NAME']+"/my_published_image/compressed", 
        CompressedImage, self.callback,  queue_size = 1)

        self.bridge = CvBridge()

    def callback(self, ros_data):
        image = self.bridge.compressed_imgmsg_to_cv2(ros_data)
        rospy.loginfo("RE IMG SUB Image size: %s", str(image.shape))
        # self.pub.publish(ros_data)


if __name__ == '__main__':
    # create the node
    node = ImageSubscriber(node_name='my_img_subscriber_node')
    # keep spinning
    rospy.spin()