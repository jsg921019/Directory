#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
import numpy as np
from darknet_ros_msgs.msg import BoundingBoxes
from xycar_motor.msg import xycar_motor

class Yolo(object):

    def __init__(self, target):
        self.detected = False
        self.target = target
        self.target_x = 0
        self.sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.callback, queue_size=1)

    def callback(self, msg):
        x = [[box.xmin, box.xmax] for box in msg.bounding_boxes if box.Class == self.target]
        if x:
            self.detected = True
            self.target_x = np.mean(x) - 320.0
            print(self.target_x)
        else:
            self.target_x = 0

    def control(self):
        angle = np.degrees(np.arctan2(self.target_x, 200)) * 5.0/2.0
        speed = 30
        return angle, speed
