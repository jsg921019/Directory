#! /usr/bin/env python
# -*- coding:utf-8 -*-

import os
import torch
import cv2

class Recorder(object):
    def __init__(self, car):
        self.fourcc = cv2.VideoWriter_fourcc(*'DIVX')
        self.curr_path = os.getcwd()
        self.save_path = self.curr_path + "/save/"
        self.video_path = self.curr_path + "/video/"
        self.make_dir()
        self.car = car
        self.fps = 10

    def make_dir(self):
        # mkdir video path
        if not os.path.isdir(self.video_path):
            os.mkdir(self.video_path)
        # mkdir save path
        if not os.path.isdir(self.save_path):
            os.mkdir(self.save_path)

    def save(self, model, fname):
        f = self.save_path + str(fname) + ".pth"
        torch.save(model.state_dict(), f)
    
    def render(self, cache, fname):
        f = self.video_path + str(fname) + ".avi"
        out = cv2.VideoWriter(f, self.fourcc, float(self.fps), (self.car.map.x, self.car.map.y))

        print(" #### start making video : " + str(fname) + ".avi #### | frame cnt : " + str(len(cache)))
        for x, y, yaw in cache:
            frame = self.car.map.img.copy()
            self.car.x, self.car.y, self.car.yaw = x, y, yaw
            self.car.draw(frame)
            cv2.putText(frame, str(fname), (100, 100), cv2.FONT_HERSHEY_DUPLEX, 1, 2, 2)
            out.write(frame)
        print(" ##### end making video : " + str(fname) + ".avi ##### ")

        out.release()