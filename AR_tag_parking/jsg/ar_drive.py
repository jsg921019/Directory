#! /usr/bin/env python

import rospy, math
import cv2
import numpy as np

from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Int32MultiArray

class Xycar(object):
    def __init__(self):
        # rospy stuffs
        self.sub = rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.callback, queue_size = 1)
        self.pub = rospy.Publisher('xycar_motor_msg', Int32MultiArray, queue_size = 1)
        self.msg = Int32MultiArray()
        self.window = cv2.namedWindow("AR tag position", cv2.WINDOW_AUTOSIZE|cv2.WINDOW_GUI_NORMAL)

        # xycar pose
        self.x, self.y, self.z = 0.0, 0.0, 0.0
        self.yaw = 0.0
        self.l = 0.0

        #xycar velocity
        self.speed = 0.0
        self.reverse = False

        # control coef
        self.k = 1.0
        self.acc = 3.0

    def callback(self, msg):
        for i in msg.markers:     
            pose = i.pose.pose
            self.x = pose.position.x
            self.y = pose.position.y
            self.z = pose.position.z
            self.yaw = euler_from_quaternion((pose.orientation.x, pose.orientation.y,
                                              pose.orientation.z, pose.orientation.w))[2]

    def control(self):

        # angle
        self.l = math.hypot(self.x, self.y)
        cte = - self.l * math.sin(self.yaw)
        yaw_term = self.yaw + math.atan2(self.x, self.y)
        cte_term = math.atan2(self.k * cte, 200.0)
        if self.reverse: yaw_term *= -1.0
        angle = 5.0 /2.0 * math.degrees(yaw_term + cte_term)

        # speed
        if self.reverse:
            target_speed = -30 + max(0, self.y - 200)*0.1
            if self.y > 300 :
                self.speed = 0
                self.reverse = False
                self.k = 1.0
            else:
                if self.speed <= target_speed + self.acc:
                    self.speed += self.acc
                elif self.speed + self.acc >= target_speed:
                    self.speed -= self.acc
                else:
                    self.speed = target_speed
        else:
            self.k = 1.0
            target_speed = 50 - max(0, 350 -self.y) * 0.13
            if 0 < self.y < 70 :
                self.speed = 0
                if not(abs(yaw_term) < math.radians(2) and abs(cte_term) < math.radians(2)):
                    self.reverse = True
                    self.k = 3.0
            else:
                if self.speed <= target_speed + self.acc:
                    self.speed += self.acc
                elif self.speed + self.acc >= target_speed:
                    self.speed -= self.acc
                else:
                    self.speed = target_speed

        # publish
        self.msg.data = [angle, self.speed]
        self.pub.publish(self.msg)

    def show(self):
        img = np.zeros((100, 500, 3))
        cv2.line(img, (20,65), (480, 65), (0, 0, 255), 1)
        cv2.line(img, (20,50), (20, 80), (0, 0, 255), 2)
        cv2.line(img, (480,50), (480, 80), (0, 0, 255), 2)
        cv2.line(img, (250,50), (250, 80), (0, 0, 255), 2)
        point = (250 + int(230*self.x/self.y), 65) if self.y else (250, 65)
        cv2.circle(img, point, 13, (0, 255, 0, 100), -1)
        cv2.putText(img, 'DX:%4d   DY:%4d   Yaw:%3.1f'%(self.x, self.y, math.degrees(self.yaw)), (20, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255))
        cv2.putText(img, '%3d pixel'%self.l, (360, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255))
        cv2.imshow("AR tag position", img)
        cv2.waitKey(1)

# main

rospy.init_node('ar_drive')
rate = rospy.Rate(10)
xycar = Xycar()
while not rospy.is_shutdown():
    xycar.control()
    xycar.show()
    rate.sleep()
cv2.destroyAllWindows()