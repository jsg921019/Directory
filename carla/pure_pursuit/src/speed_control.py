#! /usr/bin/env python
# -*- coding:utf-8 -*-

from carla_msgs import msg
import rospy
import rospkg
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from carla_msgs.msg import CarlaEgoVehicleControl
from tf.transformations import euler_from_quaternion
import numpy as np
import pickle

def sigmoid(speed):
    return 1 / (1 + math.exp(-speed))

class SpeedController(object):

    def __init__(self, ref, target_speed=4.0):
        self.ref = ref
        self.look_ahead_dist = 50
        self.target_speed = target_speed
        self.speed = None
        self.x = None
        self.y = None
        self.yaw = None
        self.nearest_idx = None
        self.kt = 0.5
        self.kb = 0.04
        self.p = 0.003
        self.sum_error = 0
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

    def init_idx(self):
        self.nearest_idx = np.argmin(np.hypot(self.x-self.ref[:,0], self.y-self.ref[:,1]))

    def update_idx(self):
        search_min = self.nearest_idx
        search_max = self.nearest_idx + 50
        self.nearest_idx += np.argmin(np.hypot(self.x-self.ref[search_min:search_max,0], self.y-self.ref[search_min:search_max,1]))

    def control_speed(self):
        error = self.speed - self.target_speed
        if error <= 0:
            self.msg.brake = 0
            self.msg.throttle = min(1, -self.kt* error - self.p*self.sum_error)
        elif error <= 1:
            self.msg.brake = 0
            self.msg.throttle = 0.5
        else:
            self.msg.brake = min(1, self.kb* (error-1))
            self.msg.throttle = 0
        self.sum_error += error
    
    def control_steer(self):
        look_ahead_point = self.ref[self.nearest_idx+self.look_ahead_dist] - np.array([self.x, self.y])
        theta = self.yaw - np.radians(90)
        rotation_matrix = np.array([[np.cos(theta), -np.sin(theta)],[np.sin(theta), np.cos(theta)]])
        x, y = np.matmul(rotation_matrix, look_ahead_point)
        self.msg.steer = 1.0* np.arctan2(2.0*2*x, x*x + y*y)

    def control(self):
        self.update_idx()
        print(self.nearest_idx)
        self.control_speed()
        self.control_steer()
        self.pub.publish(self.msg)

if __name__ == "__main__":

    # load reference path
    rospack = rospkg.RosPack()
    ref_path = rospack.get_path('pure_pursuit') + '/src/ref.pickle'
    with open(ref_path, "rb") as f:
        ref = pickle.load(f)

    # inits
    rospy.init_node("speed_control")
    speed_controller = SpeedController(ref)
    rospy.sleep(2)
    speed_controller.init_idx() 
    rate = rospy.Rate(10)

    # main loop
    while not rospy.is_shutdown():
        
        speed_controller.control()
        rate.sleep()