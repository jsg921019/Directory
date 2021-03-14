#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np
import pickle

class Recoder(object):

    def __init__(self):
        self.resolution = 0.2
        self.cache = []
        self.recording = True
        self.sub = rospy.Subscriber('/carla/ego_vehicle/odometry', Odometry, self.callback, queue_size=1)

    def callback(self, msg):
        position = msg.pose.pose.position
        #orientation = msg.pose.pose.orientation
        x = position.x
        y = position.y
        # _, _, yaw = euler_from_quaternion((orientation.x, orientation.y, orientation.z, orientation.w))
        #print(round(x,2), round(y,2), round(math.degrees(yaw), 2))
        if self.recording:
            if not self.cache:
                print('start recoding')
                self.cache.append([x, y])
                self.x0, self.y0 = x, y
            else:
                _x, _y = self.cache[-1]
                if len(self.cache) > 100 and np.hypot(x-self.x0, y-self.y0) < self.resolution:
                    self.recording = False
                    with open("ref2.pickle","wb") as f:
                        pickle.dump(np.array(self.cache), f)
                    print('finished recording')

                elif np.hypot(x-_x, y-_y) >= self.resolution:
                    recorder.cache.append([x, y])

if __name__ == "__main__":
    rospy.init_node('record')
    recorder = Recoder()
    rospy.spin()

