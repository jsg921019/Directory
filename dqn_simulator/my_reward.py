#!/usr/bin/env python

import numpy as np
import cv2

class Reward(object):
    def __init__(self,map=None, file=None):
        if map is None:
            img = cv2.imread(file)
        else:
            img = map.img
        binary = cv2.inRange(img[:,:,0], (0,), (5,))
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (130,130), (-1, -1))
        self.ref = cv2.dilate(binary, kernel, iterations = 1)

    def give_reward(self, x, y, done):

        if done:
            return -30.0
        else:
            if self.ref[int(y), int(x)] == 0 :
                return 1.0
            else:
                return -0.5

if __name__ == '__main__':
    reward = Reward(file='map.png')
    cv2.imshow('test', reward.ref)
    cv2.waitKey()