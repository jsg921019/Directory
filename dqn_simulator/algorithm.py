#!/usr/bin/env python
# -*- coding:utf-8 -*-

import numpy as np
import cv2
from assets import Car, Map
from collections import deque


#################################

def action2msg(action):
    return action*50.0 - 50.0

def msg2rad(msg):
    return np.radians(msg * -3.0 /5.0)

class Controller(object):

    def __init__(self, back = 10):
        self.back = False
        self.counter = 0
        self.max_counter = back

    def control(self, state):
        x = (state[0]-state[-1]) + (state[1]-state[-2])*np.cos(np.radians(30))
        y = state[2 ]#min(state[1:4])
        k = 2 if max(state[0],state[-1]) > 100 else 1
        if min(state[1:4]) < 30:
            k = 2

        if abs(state[1]-state[3]) > 100:
            k = 2

        angle = np.arctan2(k*x, y)

        if self.back == False:
            print(np.median(state[1:4]))
            if np.median(state[1:4]) < 15:
                self.back = True
                self.counter = self.max_counter
        else:
            if self.counter == 0:
                self.back = False

        if self.back:
            self.counter -= 1
            return -3.0 * np.degrees(angle), -50
        else:
            return -3.0 * np.degrees(angle) / 5.0, 50

class Filter(object):
    def __init__(self, ml=5):
        self.n = ml
        self.x = deque(maxlen=ml)
    
    def filter(self, angle):
        self.x.append(angle)
        return sum(self.x)/self.n

##################################

map = Map('track.png')
map.init_positions = [[50,600, np.radians(90)],
                      [73,75, np.radians(0)]]

# map = Map('loop.png')
# map.init_positions = [[100,100, np.radians(0)]]

car = Car(map)
f = Filter(10)
controller = Controller()
car.reset()

done = False
step = 0

while not done:

    step += 1

    frame = map.img.copy()
    car.draw(frame)

    state = car.measure_distance(max_dist=200, frame=frame)
    msg_angle, speed = controller.control(state)
    msg_angle = f.filter(msg_angle)
    steer_angle = msg2rad(msg_angle)
    if speed < 0:
        car.update(0, speed, dt=0.1)
    else:
        car.update(steer_angle, speed, dt=0.1)
    #done = car.check_collision()
    finished = car.check_goal()

    if finished:
        car.reset()

    if step >= 10000:
        done = True


    cv2.putText(frame, "step : " + str(step), (20, 50), cv2.FONT_HERSHEY_DUPLEX, 0.8, 2, 2)
    cv2.imshow('viewer', frame)
    if cv2.waitKey(100) == ord(' '):
        break

cv2.destroyAllWindows()