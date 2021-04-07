#! /usr/bin/env python
# -*- coding:utf-8 -*-

import cv2
import rospy
import numpy as np
from xycar_motor.msg import xycar_motor
from Timer import Timer
from XycarSensor import XycarSensor
from Detector.LaneDetector import LaneDetector
from Detector.StopLineDetector import StopLineDetector
from Detector.ObstacleDetector import ObstacleDetector
from Controller.ModeController import ModeController
from Controller.ARController import ARController
from Controller.StanleyController import StanleyController

class Xycar(object):
    '''
    Main class for AutoDriving
    '''

    def __init__(self, hz=10):
        
        self.rate = rospy.Rate(hz)
        self.pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
        self.msg = xycar_motor()

        self.timer = Timer()
        self.sensor = XycarSensor()
        yaw0 = self.sensor.init(self.rate)

        self.obstacle_detector = ObstacleDetector(self.timer)
        self.lane_detector = LaneDetector()
        self.stopline_detector = StopLineDetector()

        self.mode_controller = ModeController(yaw0, self.timer)
        self.stanley_controller = StanleyController(self.timer)
        self.ar_controller = ARController()

        steer_img = cv2.imread(rospy.get_param("steer_img"))
        self.steer_img = cv2.resize(steer_img, dsize=(0,0), fx=0.2, fy=0.2)

        self.target_lane = 'middle'
        self.control_dict = {
            'long straight' : self.stanley,
            'short straight': self.stanley,
            'obstacle': self.obstacle,
            'stopline': self.stopline,
            'curve': self.stanley,
            'findparking': self.findparking,
            'parallelparking' : self.parallelpark,
            'arparking': self.arparking,
            'poweroff' : self.poweroff
        }

    def obstacle(self):
        self.target_lane = self.obstacle_detector(self.sensor.lidar, self.sensor.angle_increment)
        if self.obstacle_detector.obstacle_counter == 4:
            print('detecting stopline...')
            self.obstacle_detector.obstacle_counter = 0
            self.mode_controller.set_mode("stopline")
        self.stanley()

    def stopline(self):
        if self.stopline_detector(self.sensor.cam):
            self.stop6s()
        else:
            self.stanley()

    def stop6s(self):
        print("stop for 5s...")
        yaws = []
        for _ in xrange(52):
            yaws.append(self.sensor.yaw)
            self.msg.angle, self.msg.speed = 0, 0
            self.pub.publish(self.msg)
            self.rate.sleep()
        print('go!')
        yaw0 = (np.mean(yaws) - np.pi) % (2*np.pi)
        self.mode_controller.set_yaw0(yaw0)
        self.mode_controller.set_mode('curve')

    def findparking(self):
        if self.timer() > 1.0:
            ranges = np.array(self.ranges)
            ranges = ranges[505/4:505/2]
            if np.count_nonzero((ranges > 0.0 ) & (ranges < 0.7)) > 20 :
                print('start parking...')
                self.mode_controller.set_mode('parallelparking')
        self.stanley()

    def parallelpark(self):
        for _ in xrange(34):
            self.msg.angle, self.msg.speed = 0, 20
            self.pub.publish(self.msg)
            self.rate.sleep()
        for _ in xrange(43):
            self.msg.angle, self.msg.speed = 50, -20
            self.pub.publish(self.msg)
            self.rate.sleep()
        for _ in xrange(43):
            self.msg.angle, self.msg.speed = -50, -20
            self.pub.publish(self.msg)
            self.rate.sleep()
        self.mode_controller.set_mode('arparking')

    def arparking(self):
        self.msg.angle, self.msg.speed = self.ar_controller(self.sensor.ar_x, self.sensor.ar_y, self.sensor.ar_yaw)
        self.pub.publish(self.msg)
        if self.msg.speed == 0:
            self.mode_controller.set_mode('poweroff')
            print('we are no.1 jayool-joohaeng developers, you know?')
        self.rate.sleep()

    def poweroff(self):
        self.msg.speed, self.msg.angle = 0, 0
        self.pub.publish(self.msg)
        self.rate.sleep()

    def stanley(self):
        angle, target = self.lane_detector(self.sensor.cam, self.target_lane)
        self.msg.angle, self.msg.speed = self.stanley_controller(angle, target, self.mode_controller.get_mode())
        self.drawsteer(self.msg.angle)
        self.pub.publish(self.msg)
        self.rate.sleep()

    def drawsteer(self, steer_angle):
        h, w = self.steer_img.shape[:2]
        cX, cY = w / 2, h / 2
        M = cv2.getRotationMatrix2D((cX, cY), -steer_angle, 1.0)
        rotated = cv2.warpAffine(self.steer_img, M, (w, h), borderValue=(255,255,255))
        cv2.imshow('steer', rotated)

    def control(self):
        '''
        main controller
        uses method based on current mode
        '''
        cv2.imshow('cam', self.sensor.cam)
        mode = self.mode_controller(self.sensor.yaw)
        self.control_dict[mode]()
        cv2.waitKey(1)