#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import rospkg
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from carla_msgs.msg import CarlaEgoVehicleControl
from tf.transformations import euler_from_quaternion
import numpy as np
import pickle

class Controller(object):

    def __init__(self, ref, hz, target_speed=12.0):

        # reference path
        self.ref = ref

        # car info
        self.max_steer_angle = 1.22173035145
        self.wb = 2.85

        # car state
        self.x = None
        self.y = None
        self.yaw = None
        self.speed = None

        # speed control params
        self.target_speed = target_speed
        self.kp = 1.0
        self.kd = 0.3
        self.ki = 0.014
        self.error_sum = 0
        self.error_prev = 0
        self.dt = 1.0 / hz

        # steer control params
        self.look_ahead_dist = 25
        self.nearest_idx = None

        # ros publishers and subscribers
        self.msg = CarlaEgoVehicleControl()
        self.sub_speed = rospy.Subscriber('/carla/ego_vehicle/speedometer', Float32, self.callback_speed, queue_size=1)
        self.sub_odom = rospy.Subscriber('/carla/ego_vehicle/odometry', Odometry, self.callback_odom, queue_size=1)
        self.pub = rospy.Publisher('/carla/ego_vehicle/vehicle_control_cmd', CarlaEgoVehicleControl, queue_size=1)

    def callback_speed(self, msg):
        self.speed = msg.data

    def callback_odom(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        self.x = position.x
        self.y = position.y
        self.yaw = euler_from_quaternion((orientation.x, orientation.y, orientation.z, orientation.w))[2]

    def control_speed(self):
        error = self.speed - self.target_speed
        d_error = (error-self.error_prev)/self.dt
        self.msg.throttle = np.clip(-self.kp * error  -self.kd*d_error - self.ki*self.error_sum, 0, 1)
        self.error_sum += error * self.dt
        self.error_sum = np.clip(self.error_sum,-40,40)
        self.error_prev = error

    def control_steer(self):
        nearest_idx = np.argmin(np.hypot(self.x-self.ref[:,0], self.y-self.ref[:,1]))
        look_ahead_idx = (nearest_idx+self.look_ahead_dist)%len(self.ref)
        look_ahead_point = self.ref[look_ahead_idx] - np.array([self.x, self.y])
        rotation_matrix = np.array([[np.cos(self.yaw), np.sin(self.yaw)],[-np.sin(self.yaw), np.cos(self.yaw)]])
        x, y = np.matmul(rotation_matrix, look_ahead_point)
        self.msg.steer = -(1/self.max_steer_angle) * np.arctan2(2 * self.wb * y, x*x + y*y)

    def control(self):
        self.control_speed()
        self.control_steer()
        self.pub.publish(self.msg)

if __name__ == "__main__":

    # parmams
    hz = 5
    ref_file = 'ref2.pickle'

    # load reference path
    rospack = rospkg.RosPack()
    ref_path = rospack.get_path('pure_pursuit') + '/src/' + ref_file
    with open(ref_path, "rb") as f:
        ref = pickle.load(f)

    # inits
    rospy.init_node("speed_control")
    speed_controller = Controller(ref=ref, hz=hz)
    rate = rospy.Rate(hz)

    # start
    for i in range(3):
        rospy.sleep(1)
        print(3-i)
    print('start!')

    # main loop
    while not rospy.is_shutdown():
        speed_controller.control()
        rate.sleep()