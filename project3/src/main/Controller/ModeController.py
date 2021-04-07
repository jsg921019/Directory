#! /usr/bin/env python
# -*- coding:utf-8 -*-

import numpy as np

class ModeController(object):
    '''
    Detects mode based on imu sensor data
    Counts laps
    '''

    def __init__(self, yaw0, timer):
        self.mode = 'long straight'
        self.timer = timer
        self.yaw0 = yaw0
        self.lap = 0
        self.lap_target = 3

    def set_mode(self, mode):
        self.mode = mode

    def set_yaw0(self, yaw0):
        self.yaw0 = yaw0

    def get_mode(self):
        return self.mode

    def __call__(self, yaw):
        '''
        updates and returns current mode
        '''
        diff_yaw = abs(yaw - self.yaw0)
        if diff_yaw > np.pi:
            diff_yaw = 2*np.pi - diff_yaw
        if self.mode == 'long straight' and  np.pi/2.0 - 0.1 < diff_yaw < np.pi/2.0 + 0.1:
            self.mode = 'short straight'
            self.timer.update()
        elif self.mode == 'short straight' and  np.pi - 0.15 < diff_yaw < np.pi + 0.15:
            print('detecting obstacle...')
            self.timer.update()
            self.mode = 'obstacle'
        elif self.mode == 'curve' and  diff_yaw < 0.05:
            self.lap += 1
            print('finish lap {}'.format(self.lap))
            self.timer.update()
            if self.lap < self.lap_target:
                self.mode = 'long straight'
                #self.stanley_k = 0.7
            else:
                print('finding parking lot...')
                self.mode = 'findparking'
        return self.mode