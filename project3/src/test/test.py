#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
from Xycar_test import Xycar

rospy.init_node('lane_detection')

xycar = Xycar()

while not rospy.is_shutdown():
    xycar.control()
