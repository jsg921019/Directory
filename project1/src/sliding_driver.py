#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from std_msgs.msg import Int32
from xycar_motor.msg import xycar_motor
import numpy as np
import cv2


######## 조향각 평균 필터링 ########

class MovingAverage:
    def __init__(self, n):
        self.samples = n
        self.data = [0]*n
        self.weights = list(range(1, n + 1))

    def add_sample(self, new_sample):
        if len(self.data) < self.samples:
            self.data.append(new_sample)
        else:
            self.data = self.data[1:] + [new_sample]

    def get_mm(self):
        return float(sum(self.data)) / len(self.data)

    def get_wmm(self):
        s = 0
        for i, x in enumerate(self.data):
            s += x * self.weights[i]
        return float(s) / sum(self.weights[:len(self.data)])



######## 설정 변수 #########

Kp = 0.4
Kd = 1
Ki = 0
speed = 50
track_error = {"sum":0, "prev":0}



######## 객체 생성 #########
steer_avr = MovingAverage(10)
motor_control = xycar_motor()



######## 함수 ########

def callback(msg):
    
    error_diff = msg.data - track_error["prev"]
    track_error["sum"] += msg.data
    track_error["prev"] = msg.data
    motor_control.angle = Kp*msg.data + Kd*error_diff + Ki*track_error["sum"]
    motor_control.speed = speed
    pub.publish(motor_control)

    
def draw_steer(steer_angle):

    arrow_pic = cv2.imread('steer_arrow.png', cv2.IMREAD_COLOR)

    origin_Height = arrow_pic.shape[0]
    origin_Width = arrow_pic.shape[1]

    steer_wheel_center = origin_Height * 0.74

    matrix = cv2.getRotationMatrix2D((origin_Width/2, steer_wheel_center), (steer_angle) * 2.5, 0.7)    
    arrow_pic = cv2.warpAffine(arrow_pic, matrix, (origin_Width+60, origin_Height))
    arrow_pic = cv2.resize(arrow_pic, dsize=(0, 0), fx=0.5, fy=0.5)

    cv2.imshow('steer', arrow_pic)




######## main ########

if __name__ == "__main__":

    rospy.init_node('driver')

    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    sub = rospy.Subscriber('error', Int32, callback, queue_size=1)
    while not rospy.is_shutdown():
        draw_steer(steer_avr.get_mm())
        if cv2.waitKey(33) & 0xFF == ord('q'):
            break
    #rospy.spin()
