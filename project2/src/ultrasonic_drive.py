#! /usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np

class Ultrasonic(object):

    def __init__(self):
        self.backward = False
        self.stop = False
        self.stop_counter = 0
        self.stop_counter_max = 10
        self.speed = 35

    def pre_process(self, ultrasonic):
        return np.clip(ultrasonic, [30, 0, 0, 0, 30, 0, 0, 0], [100, 250, 300, 250, 100, 200, 1000, 1000])

    def drive_go(self, ultrasonic):
        r_left, left, center, right, r_right, _, _, _ = self.pre_process(ultrasonic)
        speed = self.speed if center <= 50 else self.speed + 10
        angle = (-left - 15 + right + 15)*0.8 - r_left * 1.0 + r_right * 1.0
        return angle, speed

    def drive_stop(self):
        self.stop_counter -= 1
        if self.stop_counter == 0:
            self.stop = False
        return 0, -self.speed

    def drive_backward(self):
        self.stop_counter -= 1
        if self.stop_counter == 0:
            self.stop = False
        return 50, -self.speed -5

    def control(self, ultrasonic):

        if self.stop == False and ultrasonic[2] < 35:
            self.stop = True
            self.stop_counter = self.stop_counter_max

        if self.stop:
            if self.backward:
                return self.drive_backward()
            else:
                return self.drive_stop()

        else:
            return self.drive_go(ultrasonic)
