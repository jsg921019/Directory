#! /usr/bin/env python

import rospy, math
import cv2
import numpy as np

from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Int32MultiArray

class Xycar(object):
    def __init__(self, hz):
        # rospy stuffs
        self.sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.callback, queue_size = 1)
        self.pub = rospy.Publisher('xycar_motor_msg', Int32MultiArray, queue_size = 1)
        self.msg = Int32MultiArray()
        self.hz = hz
        self.rate = rospy.Rate(hz)

        # xycar pose
        self.x, self.y, self.z = 0.0, 0.0, 0.0
        self.yaw = 0.0

        # control coef
        self.k = 1.0
        self.stanley_dist = 400
        self.stop_dist = 70

        # etc
        self.state = "stop"
        self.shift_counter = 0
        self.shift_counter_max = 0
        self.shift_direction = True
        self.stop_iter = 60

    def callback(self, msg):
        for i in msg.markers:     
            pose = i.pose.pose
            self.x = pose.position.x
            self.y = pose.position.y
            self.z = pose.position.z
            self.yaw = euler_from_quaternion((pose.orientation.x, pose.orientation.y,
                                              pose.orientation.z, pose.orientation.w))[2]

    def stop(self, x, y, yaw):
        if y > self.stop_dist + 100:
            self.state = "go"
            print("state : go")
            self.go(x, y, yaw)
        else:
            self.msg.data = [0, 0]
            self.pub.publish(self.msg)
            self.rate.sleep()

    def go(self, x, y, yaw):
        if y < self.stop_dist:
            print("checking...")
            self.check(x, y, yaw)
        elif y < self.stanley_dist:
            l = math.hypot(x, y)
            cte = - l * math.sin(yaw)
            yaw_term = self.yaw + math.atan2(x, y)
            cte_term = math.atan2(self.k * cte, 100.0)
            angle = 5.0 /2.0 * math.degrees(yaw_term + cte_term)
            self.msg.data = [angle, 20]
            self.pub.publish(self.msg)
            self.rate.sleep()
        else:
            angle = math.atan2(x, y)
            angle = 5.0 /2.0 * math.degrees(angle)
            self.msg.data = [angle, 30]
            self.pub.publish(self.msg)
            self.rate.sleep()

    def check(self, x, y, yaw):
        yaw_term = yaw + math.atan2(x, y)
        if abs(yaw_term) > math.radians(5):
            print("reverse needed")
            self.state = "shift"
            self.shift_counter_max = int(abs(yaw_term) * 15 * self.hz)
            self.shift_counter = self.shift_counter_max
            if yaw_term > 0:
                self.shift_direction = False
            else:
                self.shift_direction = True
            self.transition()
        else:
            self.state = "stop"
            print("finished")
            self.stop(x, y, yaw)

    def shift(self, x, y, yaw):
        if self.shift_counter > 0:
            if self.shift_counter > self.shift_counter_max/2:
                self.msg.data = [50 if self.shift_direction else -50, -20]
                self.pub.publish(self.msg)
                self.rate.sleep()
            else:
                self.msg.data = [-50 if self.shift_direction else 50, -20]
                self.pub.publish(self.msg)
                self.rate.sleep()
            self.shift_counter -= 1
        else:
            print("reverse needed")
            self.state = "go"
            self.transition()

    def transition(self):
        for _ in range(self.stop_iter):
            self.msg.data = [0, 0]
            self.pub.publish(self.msg)
            self.rate.sleep()

    def control(self):

        x, y, yaw = self.x, self.y, self.yaw

        if self.state == "go":
            self.go(x, y, yaw)
        elif self.state == "shift":
            self.shift(x, y, yaw)
        elif self.state == "finish":
            self.finish(x, y, yaw)
        elif self.state == "stop":
            self.stop(x, y, yaw)
        else:
            print("unknown state : ", self.state)

# main
hz = 30
rospy.init_node('ar_drive')
xycar = Xycar(hz)

while not rospy.is_shutdown():
    xycar.control()