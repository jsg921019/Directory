#!/usr/bin/env python

import cv2, time, rospy
import numpy as np
from pyzbar import pyzbar
from ultrasonic_drive_3 import *
from std_msgs.msg import Int32MultiArray
from xycar_motor.msg import xycar_motor


rospy.init_node('ultra_driver')
pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size = 1)
motor_msg = xycar_motor()

cap = cv2.VideoCapture(0)

angle = 0
speed = 0


while not rospy.is_shutdown():
    mode = ""
    ret, image = cap.read()
   
    qrcodes = pyzbar.decode(image)
    
    for qrcode in qrcodes:
        (x, y, w, h) = qrcode.rect
        cv2.rectangle(image, (x, y), (x+w, y+h), (0, 0, 255), 2)

        mode = qrcode.data

        cv2.putText(image, str(mode), (x, y-6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)

        print(mode)
        

    if mode == "1 algorithm drive_avoid_obstacle ":
        motor_msg.angle, motor_msg.speed = main()
        print(motor_msg.angle)
        print(motor_msg.speed)
	pub.publish(motor_msg)
   


    cv2.imshow('camera', image)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


