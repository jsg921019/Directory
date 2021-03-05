#! /usr/bin/env python
# -*- coding:utf-8 -*-

import cv2
import numpy as np

def get_pixels_line(p1, p2):
    step = max(abs(p1-p2)) + 1
    x = np.linspace(p1[0], p2[0], step).astype(np.int32)
    y = np.linspace(p1[1], p2[1], step).astype(np.int32)
    return x, y

def get_pixels_rect(points):
    p1s = points
    p2s = np.roll(points,1, axis=0)
    x = []
    y = []
    for p1, p2 in zip(p1s, p2s):
        _x, _y = get_pixels_line(p1, p2)
        x.append(_x)
        y.append(_y)
    return np.hstack(x), np.hstack(y)

class Map(object):
    def __init__(self, file):
        self.img = cv2.imread(file)
        self.y, self.x, _ = self.img.shape
        self.init_positions = [[154, 84, np.radians(0)],
                               [154, 84, np.radians(20)],
                               [154, 84, np.radians(-20)],
                               [416, 235, np.radians(180)],
                               [416, 235, np.radians(160)],
                               [416, 235, np.radians(200)]]

class Car(object):
    def __init__(self, map, x=0, y=0, yaw=0):
        self.map = map
        self.x = x
        self.y = y
        self.yaw = yaw % (2.0 * np.pi)
        self.width = 27.0
        self.length = 55.0
        self.wb = 35.0
        self.max_steer_angle = np.radians(25)
        self.box = np.array([[-9.0, self.width/2], [self.length - 9.0, self.width/2],
                             [self.length - 9.0, -self.width/2], [-9.0, -self.width/2]])
        self.hitbox = np.array([[-13.0, self.width/2 + 5], [52.0, self.width/2 + 5],
                                [52.0, -self.width/2 - 5], [-13.0, -self.width/2 - 5]])
        self.sensor_position = np.array([[20, -self.width/2 - 1], [45, -self.width/4], [46, 0], [45, self.width/4], [20, self.width/2 + 1]])
        self.sensor_angle = np.radians([90, 30, 0, -30, -90])

    def update(self, steer, speed, dt):
        steer = np.clip(steer, -self.max_steer_angle, self.max_steer_angle)
        self.x += speed * np.cos(self.yaw) * dt
        self.y -= speed * np.sin(self.yaw) * dt
        self.yaw += speed / self.wb * np.tan(steer) * dt
        self.yaw = self.yaw % (2.0 * np.pi)

    def get_pose(self):
        return [self.x, self.y, self.yaw]

    def transform(self, points):
        rotation_matrix = np.array([[np.cos(self.yaw), -np.sin(self.yaw)],
                                    [np.sin(self.yaw), np.cos(self.yaw)]])
        translation_matrix = np.array([self.x, self.y])
        transformed = np.matmul(points, rotation_matrix) + translation_matrix
        return transformed.astype(np.int32)

    def draw(self, frame):
        rect = self.transform(self.box)
        cv2.fillPoly(frame, [rect], (200, 0, 0), 4)
        #cv2.circle(frame, (int(self.x), int(self.y)), 5, (0,0,255), -1)
        #cv2.circle(frame, (int(self.x+ self.wb), int(self.y)), 5, (0,0,255), -1)

    def draw_hitbox(self, frame):
        hitbox = self.transform(self.hitbox)
        x, y = get_pixels_rect(hitbox)
        frame[y, x] = (0, 255, 0)

    def check_collision(self):
        hitbox = self.transform(self.hitbox)
        x, y = get_pixels_rect(hitbox)
        if np.any(self.map.img[y, x, 1] == 0):
            return True
        else:
            return False

    def check_goal(self):
        if 150 <= self.map.img[int(self.y), int(self.x), 1] < 200 :
            return True
        else:
            return False

    def measure_distance(self, frame=None, max_dist = None):
        if max_dist is None:
            max_dist = self.map.x + self.map.y
        measurement = []
        sensor_rot_position = self.transform(self.sensor_position)
        for p1, theta in zip(sensor_rot_position, self.sensor_angle):
            p2 = p1 + max_dist * np.array([np.cos(self.yaw+theta), -np.sin(self.yaw+theta)])
            x, y = get_pixels_line(p1, p2)
            for i, (_x, _y) in enumerate(zip(x, y)):
                if _x == 0 or _x == self.map.x-1 or _y == 0 or _y == self.map.y-1:
                    if frame is not None:
                        frame[y[:i], x[:i]] = (0, 255, 0)
                    measurement.append(max_dist)
                    break
                if self.map.img[_y, _x, 2] < 5:
                    if frame is not None:
                        frame[y[:i], x[:i]] = (0, 255, 0)
                    measurement.append(np.hypot(p1[0]-_x, p1[1]-_y))
                    break
            else:
                if frame is not None:
                    frame[y, x] = (0, 255, 0)
                measurement.append(max_dist)

        return np.array(measurement)

    def reset(self, position=None):
        if position == None:
            position = np.random.randint(0, len(self.map.init_positions))
        self.x, self.y, self.yaw = self.map.init_positions[position]

if __name__ == "__main__" :

    map = Map('map.png')
    car = Car(map)
    for i in range(len(map.init_positions)):
        car.reset(position=i)
        frame = map.img.copy()
        car.draw(frame)
        car.draw_hitbox(frame)
        state = car.measure_distance(frame, 200.0)
        print(state)
        cv2.imshow('test', frame)
        if cv2.waitKey(3000) == ord(' '):
            break