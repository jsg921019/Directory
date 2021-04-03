#! /usr/bin/env python

import rospy
import numpy as np
from collections import deque
from tf.transformations import euler_from_quaternion
from ar_track_alvar_msgs.msg import AlvarMarkers


class AR(object):
    def __init__(self):

        # initialize
        self.detected = False

        # rospy stuffs
        self.sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.callback, queue_size = 1)

        # median filter
        self.xs, self.ys, self.yaws = deque(maxlen=5), deque(maxlen=5), deque(maxlen=5)

        # xycar pose
        self.x, self.y, self.z, self.yaw = 0.0, 0.0, 0.0, 0.0
        self.reverse = False

        # control coef
        self.k = 2.0

    def callback(self, msg):
        self.detected = True
        for i in msg.markers:     
            pose = i.pose.pose
            self.xs.append(pose.position.x)
            self.ys.append(pose.position.z)
            self.yaws.append(euler_from_quaternion((pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))[1])
            self.y = np.median(self.ys)
            self.x = np.median(self.xs)
            self.yaw = np.median(self.yaws)

    def control(self):

        # angle control
        cte = -self.y * np.sin(self.yaw) + self.x * np.cos(self.yaw)    
        yaw_term = self.yaw
        cte_term = np.arctan2(self.k * cte, 1.0)
        if self.reverse:
            yaw_term *= -1.0
            angle = 5.0 /2.5 * np.degrees(yaw_term + cte_term)
        else:
            angle = 5.0 /2.5 * np.degrees(yaw_term + cte_term)

        # speed control
        if self.reverse:
            if self.y > 1.2:
                speed = 35
                self.reverse = False
                self.k = 1.0
            else:
                speed = -35
        else:
            if 0 < self.y < 0.6:
                # reverse condition
                if abs(yaw_term) > np.radians(10) or  abs(self.x) > 0.05:
                   speed = -35
                   self.reverse = True
                   self.k = 1.0
                else:
                    speed, angle = 0, 0
            else:
                speed = 35

        return angle, speed
