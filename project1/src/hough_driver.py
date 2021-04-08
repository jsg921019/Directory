#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Int32MultiArray
from xycar_motor.msg import xycar_motor
import numpy as np
import cv2


######## 필터 ########

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

# 곡선구간 속력, 커브구간 속력
speed_min, speed_max = 20, 25

# 직선구간 계수, 커브구간 계수
Kp_line , Kp_curve  = 0.15, 0.35

# 직선/커브 변환 관련 카운터
count_max_straight, count_max_curve = 40, 30

# state : 1이면 직선구간 0이연 커브구간
speed_par= {"state":1, "count_if_straight":count_max_straight, "count_if_curve":count_max_curve}




######## 객체 생성 #########

filter_error = MovingAverage(10) 
motor_control = xycar_motor()



######## 콜백 함수 ########

def callback(msg):
    error, is_straight = msg.data
    filter_error.add_sample(error)

    # 현재 직선 상태인 경우
    if speed_par["state"] == 1:
        motor_control.speed = speed_max
        motor_control.angle = Kp_line * filter_error.get_mm()

        # 중앙선이 인식되었으면 카운터 리셋
        if is_straight == 1:
            speed_par["count_if_straight"] = count_max_straight 

        # 중앙선 인식안된경우 카운터 1 감소, 카운터가 이미 0이면 커브 상태로 전환
        else:
            if speed_par["count_if_straight"] == 0 :
                speed_par["state"] = 0
                speed_par["count_if_curve"] = count_max_curve
                print("state : curve")  
            else:
                speed_par["count_if_straight"] -= 1

    # 현재 커브 상태인 경우            
    else:
        motor_control.speed = speed_min
        motor_control.angle = Kp_curve * filter_error.get_mm()

        # 중앙선 인식 안된 경우 카운터 초기화
        if is_straight == 0:
            speed_par["count_if_curve"] = count_max_curve

        # 중앙성 인식 된 경우 카운터 1 감소, 카운터가 이미 0이면 직진 상태로 전환
        else:
            if speed_par["count_if_curve"] == 0 :
                speed_par["state"] = 1
                speed_par["count_if_straight"] = count_max_straight
                print("state : straight") 
            else:
                speed_par["count_if_curve"] -= 1
                     
    pub.publish(motor_control)

    
######## main ########

if __name__ == "__main__":

    rospy.init_node('driver')

    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    sub = rospy.Subscriber('error', Int32MultiArray, callback, queue_size=1)
    rospy.spin()