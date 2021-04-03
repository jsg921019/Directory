#! /usr/bin/env python
# -*- coding:utf-8 -*-

import cv2
import rospy
import numpy as np
from XycarVision_test import XycarVision

class Xycar(object):

    def __init__(self, hz=10):
        self.xv = XycarVision()
        self.rate = rospy.Rate(hz)
        self.stanley_k = 1.0
        steer_img = cv2.imread(rospy.get_param("steer_img"))
        self.steer_img = cv2.resize(steer_img, dsize=(0,0), fx=0.2, fy=0.2)
        self.init()

    def init(self):
        # ready usb_cam
        while self.xv.img is None:
            self.rate.sleep()
        print("usb_cam ready")

    def drawsteer(self, steer_angle):
        h, w = self.steer_img.shape[:2]
        cX, cY = w / 2, h / 2
        M = cv2.getRotationMatrix2D((cX, cY), -steer_angle, 1.0)
        rotated = cv2.warpAffine(self.steer_img, M, (w, h), borderValue=(255,255,255))
        cv2.imshow('steer', rotated)

    def stanley(self):
        angle, target = self.xv.update()
        yaw_term = angle
        cte = (target - 320) * 1.9 / 650.0
        cte_term = np.arctan2(self.stanley_k * cte, 3.0)
        steer_angle = np.degrees(0.4 * yaw_term + cte_term) * 5.0/3.0
        return steer_angle

    def control(self):
        steer_angle = self.stanley()
        self.drawsteer(steer_angle)
        cv2.waitKey(1)
        self.rate.sleep()