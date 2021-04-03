#! /usr/bin/env python
# -*- coding:utf-8 -*-

import cv2
import time
import rospy
import numpy as np
from xycar_motor.msg import xycar_motor
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion
from XycarVision import XycarVision


class Xycar(object):
    def __init__(self, hz=10):

        self.xv = XycarVision()
        self.rate = rospy.Rate(hz)

        # subscribers
        self.sub_lidar = rospy.Subscriber("scan", LaserScan, self.callback_lidar, queue_size=1)
        self.sub_imu = rospy.Subscriber('imu', Imu, self.callback_imu, queue_size=1)
        self.sub_ar = rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.callback_ar, queue_size = 1)

        # motor publisher
        self.pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
        self.msg = xycar_motor()

        # initial state
        self.speed = 50
        self.stanley_k = 0.7
        self.mode = 'long straight'
        self.follow = 'middle'
        self.timer = time.time()

        # ridar related
        self.ranges = None
        self.angle_increments = None
        self.lidar_sleep_t = 1.4

        # imu related
        self.yaw = None
        self.yaw0 = None

        # obstacle related
        self.obstacle_counter = 0
        self.obs_dict = {1:1.0, 2:2.6, 3:4.0}

        # parking relatated
        self.reverse = False
        self.ahead = 0

        # lap counter
        self.lap = 0
        self.lap_target = 1

        # control methods
        self.control_dict = {
            'long straight' : self.long_straight,
            'short straight': self.short_straight,
            'obstacle': self.obstacle,
            'stopline': self.stopline,
            'curve': self.stanley,
            'findparking': self.findparking,
            'parallelparking' : self.parallelpark,
            'arparking': self.arparking,
            'poweroff' : self.poweroff
        }

        self.init()

    def callback_lidar(self, msg):
        self.ranges = msg.ranges
        self.angle_increment = msg.angle_increment

    def callback_imu(self, data):
        _, _, yaw = euler_from_quaternion((data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w))
        self.yaw = yaw % (2*np.pi)

    def callback_ar(self, msg):
        for i in msg.markers:     
            pose = i.pose.pose
            self.ar_x = pose.position.x
            self.ar_y = pose.position.z
            self.ar_yaw = euler_from_quaternion((pose.orientation.x, pose.orientation.y,
                                              pose.orientation.z, pose.orientation.w))[1]

    def init(self):

        # ready usb_cam
        while self.xv.img is None:
            self.rate.sleep()
        
        print("usb_cam ready")
        
        # ready lidar
        while self.ranges is None:
            self.rate.sleep()

        print("lidar ready")

        # ready imu
        while self.yaw is None:
            self.rate.sleep()

        print("imu ready")

        # set initial yaw
        yaws = []
        for _ in xrange(11):
            yaws.append(self.yaw)
            self.rate.sleep()
        self.yaw0 = np.median(yaws)
        print("initial yaw = {}".format(self.yaw0))
        self.rate.sleep()

    def update_mode(self):

        diff_yaw = abs(self.yaw - self.yaw0)
        if diff_yaw > np.pi:
            diff_yaw = 2*np.pi - diff_yaw

        if self.mode == 'long straight' and  np.pi/2.0 - 0.1 <diff_yaw < np.pi/2.0 + 0.1:
            self.mode = 'short straight'
            self.timer = time.time()

        elif self.mode == 'short straight' and  np.pi - 0.15 < diff_yaw < np.pi + 0.15:
            print('detecting obstacle...')
            self.timer = time.time()
            self.mode = 'obstacle'
            self.speed = 35

        elif self.mode == 'curve' and  diff_yaw < 0.05:
            self.lap += 1
            print('finish lap {}'.format(self.lap))
            self.timer = time.time()
            if self.lap < self.lap_target:
                self.mode = 'long straight'
                self.stanley_k = 0.7
            else:
                print('finding parking lot...')
                self.mode = 'findparking'
                self.timer = time.time()
                
    def obstacle(self):

        if self.obstacle_counter == 0 and (time.time()-self.timer > 0.35):
            ranges = np.array(self.ranges)
            ranges[:505/4] = 0.0
            ranges[505*3/4:] = 0.0
            deg = np.arange(505) * self.angle_increment - 252 * self.angle_increment
            mask = (np.abs(ranges * np.sin(deg)) < 0.4) & (0.2 < ranges * np.cos(deg)) & (ranges * np.cos(deg) < 0.4)
            filtered = np.where(mask, ranges, 0.0)
            nz = np.nonzero(filtered)[0]
            if len(nz) > 5:
                self.stanley_k = 1.0
                if np.median(nz) > 505/2:
                    self.follow = 'right'
                else:
                    self.follow = 'left'
                print('avoid to', self.follow)
                self.timer = time.time()
                self.obstacle_counter += 1

        elif self.obstacle_counter != 0:
            if time.time() - self.timer > self.obs_dict[self.obstacle_counter]:
                if self.obstacle_counter == 3:
                    self.timer = time.time()
                    self.obstacle_counter = 0
                    self.follow = 'middle'
                    self.mode = "stopline"
                    self.speed = 40
                    print("detecting stopline...")
                else:
                    self.follow = 'left' if self.follow == 'right' else 'right'
                    print('avoid to', self.follow)
                    self.obstacle_counter += 1
        self.stanley()

    def stop6s(self):
        print("stop for 5s...")
        yaws = []
        for _ in xrange(52):
            yaws.append(self.yaw)
            self.msg.angle, self.msg.speed = 0, 0
            self.pub.publish(self.msg)
            self.rate.sleep()
        print('go!')
        self.yaw0 = (np.mean(yaws) - np.pi) % (2*np.pi)
        self.speed = 50
        self.mode = 'curve'

    def findparking(self):
        if time.time() - self.timer > 1.0:
            self.speed = max(self.speed - 0.6, 20)
            ranges = np.array(self.ranges)
            ranges = ranges[505/4:505/2]
            if np.count_nonzero((ranges > 0.0 ) & (ranges < 0.7)) > 20 :
                self.mode = 'parallelparking'
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
        self.mode = 'arparking'

    def arparking(self):

	    # direction
        if self.reverse:
            if self.ar_y > 0.4:
                self.reverse = False
                self.ahead = (self.ar_x+0.01) * 150
        else:
            if self.ar_y < 0.24:
                self.reverse = True
                self.tmp_yaw = self.ar_yaw

        # termination
        if -0.02 < self.ar_x + 0.01 < 0.02 and 0.24 < self.ar_y < 0.27 and abs(self.ar_yaw) < 0.035:
            self.msg.angle, self.msg.speed = 0, 0
            self.pub.publish(self.msg)
            self.rate.sleep()
            self.mode = 'poweroff'
            print('we are no.1 jayool-joohaeng developers, you know?')
            return

        if self.reverse:
            angle = -300 * (self.ar_yaw - 0.05) 
        else:
            angle = -self.ahead +20 if self.ar_y < 0.35 else self.ahead + 20     
        speed = -16 if self.reverse else 16
        self.msg.angle, self.msg.speed = angle, speed
        self.pub.publish(self.msg)
        self.rate.sleep()

    def poweroff(self):
        self.msg.speed, self.msg.angle = 0, 0
        self.pub.publish(self.msg)
        self.rate.sleep()        

    def stopline(self):
        if time.time() - self.timer > 2.8:
            self.speed = max(self.speed - 2.0, 20)
        if self.xv.detect_stopline_contour():
            self.stop6s()
        else:
            self.stanley()

    def short_straight(self):
        if time.time() - self.timer > 2.5:
            self.speed = max(self.speed - 0.4, 45)
        self.stanley()

    def long_straight(self):
        if time.time() - self.timer > 1.0:
            self.speed = min(self.speed + 1.0, 50)
        self.stanley() 

    def stanley(self):
        angle, target = self.xv.update(self.follow)
        yaw_term = angle
        cte = (target-320) * 1.9/650.0
        cte_term = np.arctan2(self.stanley_k*cte,3.0)
        self.msg.angle, self.msg.speed = np.degrees(0.4*yaw_term+cte_term)*5.0/3.0 + 2, self.speed
        self.pub.publish(self.msg)
        #print(self.msg.angle, self.speed)
        self.rate.sleep()

    def drawsteer(self, steer_angle):
        h, w = self.steer_img.shape[:2]
        cX, cY = w / 2, h / 2
        M = cv2.getRotationMatrix2D((cX, cY), -steer_angle, 1.0)
        rotated = cv2.warpAffine(self.steer_img, M, (w, h), borderValue=(255,255,255))
        cv2.imshow('steer', rotated)

    def control(self):
        self.update_mode()
        self.control_dict[self.mode]()
        cv2.waitKey(1)