#! /usr/bin/env python
# -*- coding: utf-8 -*-


# imports ---------------------------#

import rospy
from pyzbar import pyzbar
from cv_bridge import CvBridge

from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Image
from xycar_motor.msg import xycar_motor

from filter import Filter
from ultrasonic_drive import Ultrasonic
from dqn_drive import DQN
from yolo_drive import Yolo
from ar_drive import AR


# main class -------------------------#

class Xycar(object):

    def __init__(self, mode='start', hz=10):

        self.rate = rospy.Rate(hz)
        self.mode = mode

        # ultrasonic
        self.ultrasonic = None
        self.filter = Filter()
        self.sub_ultrasonic = rospy.Subscriber('xycar_ultrasonic', Int32MultiArray, self.callback_ultrasonic)

        # image
        self.img = None
        self.bridge = CvBridge()
        self.sub_img = rospy.Subscriber("/usb_cam/image_raw/", Image, self.callback_img)

        # qrcode
        self.qr_dict = {
            '1' : 'ultrasonic',
            '2' : 'turn',
            '3' : 'dqn',
            '4' : 'bottle',
            '5' : 'yolo',
            '6' : 'pottedplant',
            '7' : 'command1',
            '8' : 'command2',
            '9' : 'park'}

        # motor
        self.pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size = 1)
        self.msg = xycar_motor()

        # controllers
        self.controller_ultrasonic = Ultrasonic()
        self.controller_DQN = DQN()
        self.controller_yolo = Yolo('bottle')
        self.controller_ar = AR()
        self.control_dict = {
            'start' : self.start,
            'ultrasonic' : self.ultrasonic_control,
            'turn' : self.turn_control,
            'dqn' : self.dqn_control,
            'yolo' : self.yolo_control,
            'park' : self.ar_control}

    def callback_ultrasonic(self, msg):
        self.ultrasonic = self.filter.filter(msg.data)

    def callback_img(self, msg):
        self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def sleep(self):
        self.rate.sleep()

    def read(self, target):
        if self.img is None:
            return
        qrcodes = pyzbar.decode(self.img)
        if qrcodes:
            text = qrcodes[0].data
            if self.qr_dict[text[0]] == target:
                self.mode = target
                if target == 'turn':
                    self.controller_ultrasonic.backward = True
                    self.controller_ultrasonic.speed = 30
                    self.controller_ultrasonic.stop_counter_max = 30
                elif target == 'yolo':
                    self.controller_yolo.detected = False
                elif target == 'ar':
                    self.controller_ar.detected = False

    def start(self):
        self.read('ultrasonic')
        return 0, 35

    def ultrasonic_control(self):
        self.read('turn')
        return self.controller_ultrasonic.control(self.ultrasonic)

    def turn_control(self):
        self.read('dqn')
        return self.controller_ultrasonic.control(self.ultrasonic)

    def dqn_control(self):
        self.read('yolo')
        return self.controller_DQN.control(self.ultrasonic)

    def yolo_control(self):
        self.read('park')
        if self.controller_yolo.detected:
            return self.controller_yolo.control()
        else:
            return self.controller_ultrasonic.drive_go(self.ultrasonic)[0], 30

    def ar_control(self):
        if self.controller_ar.detected:
            return self.controller_ar.control()
        else:
            return -10, 30

    def control(self):
        self.msg.angle, self.msg.speed = self.control_dict[self.mode]()
        self.pub.publish(self.msg)



# main ---------------------------#

if __name__ == '__main__':

    rospy.init_node('rally')
    xycar = Xycar(mode='start')

    for _ in range(30):
        xycar.sleep()

    while not rospy.is_shutdown():
        xycar.control()
        xycar.sleep()