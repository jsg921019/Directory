#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import cv2, random, math
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge


######## 고정값 ########

Width, Height  = 640, 480
red, green, blue = (0, 0, 255), (0, 255, 0), (255, 0, 0)


######## 설정값 ########

# ROI 크기 설정
Offset, Gap = 320, 40

# gaussianBlur 설정
kernel_size = (5,5)

# Canny 설정
low_threshold, high_threshold = 60, 70

# HoughLineP 설정
hough_threshold, min_length, min_gap = 30, 30, 10

# 기울기 필터링 범위
low_slope_threshold , high_slope_threshold = 0, 2

# 왼쪽 오른쪽 구별 설정
divide_margin = 120

# 이미지 초기값
img_from_cam = np.empty(shape=[0])


######## 함수 ########

def img_callback(data):
    global img_from_cam
    img_from_cam = bridge.imgmsg_to_cv2(data, "bgr8")

def draw_lines(frame, x1, y1, x2, y2, color=None, Offset=Offset):
        if color is None:
            color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
        cv2.line(frame, (x1, y1+Offset), (x2, y2+Offset), color, 2)

def draw_rectangle(img, lpos, rpos, offset=0):

    center = (lpos + rpos) / 2

    cv2.rectangle(img, (lpos - 5, 15 + offset), (lpos + 5, 25 + offset), green, 2)
    cv2.rectangle(img, (rpos - 5, 15 + offset), (rpos + 5, 25 + offset), green, 2)
    cv2.rectangle(img, (center-5, 15 + offset), (center+5, 25 + offset), green, 2)    
    cv2.rectangle(img, (315, 15 + offset), (325, 25 + offset), red, 2)
    cv2.rectangle(img, (0, Offset), (Width - 1, Offset + Gap), blue, 3)

def divide_left_right(frame, lines):

    left, right = [], []

    for x1, y1, x2, y2 in lines[:, 0]:

        m = float("inf") if x1 == x2 else float(y2-y1) / float(x2-x1)

        if not low_slope_threshold < abs(m) < high_slope_threshold:
            continue

        b = y1 - m * x1
        if (m < 0) and (x2 < Width/2 - divide_margin):
            left.append([m, b])
            draw_lines(frame, x1, y1, x2, y2)

        elif (m > 0) and (x1 > Width/2 + divide_margin):
            right.append([m, b])
            draw_lines(frame, x1, y1, x2, y2)

    return left, right

def get_line_pos(line, left=True):

    x1 = x2 = pos = 0 if left else Width
    
    if line:
        m, b = np.median(line, 0)
        pos = ((Gap / 2) - b) / m
        b += Offset
        x1, x2 = (Height - b) / float(m), ((Height/2) - b) / float(m)

    cv2.line(image, (int(x1), Height), (int(x2), (Height/2)), (255, 0,0), 3)
    return int(pos)

def process_image(frame):

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, kernel_size, 0)
    canny = cv2.Canny(blur, low_threshold, high_threshold)
    roi = canny[Offset : Offset+Gap, 0 : Width]
    cv2.imshow("roi", roi)

    lines = cv2.HoughLinesP(roi, 1, math.pi/180, hough_threshold, min_length, min_gap)

    if lines is None:
        return 320

    left_line, right_line = divide_left_right(frame, lines)
    lpos, rpos = get_line_pos(left_line, left=True), get_line_pos(right_line, left=False)

    cv2.line(frame, (230, 235), (410, 235), (255,255,255), 2)
    draw_rectangle(frame, lpos, rpos, offset=Offset)

    return (lpos + rpos) / 2

def draw_steer(image, steer_angle):
    global Width, Height, arrow_pic

    arrow_pic = cv2.imread('steer_arrow.png', cv2.IMREAD_COLOR)

    origin_Height = arrow_pic.shape[0]
    origin_Width = arrow_pic.shape[1]
    steer_wheel_center = origin_Height * 0.74
    arrow_Height = Height/2
    arrow_Width = (arrow_Height * 462)/728

    matrix = cv2.getRotationMatrix2D((origin_Width/2, steer_wheel_center), (steer_angle) * 2.5, 0.7)    
    arrow_pic = cv2.warpAffine(arrow_pic, matrix, (origin_Width+60, origin_Height))
    arrow_pic = cv2.resize(arrow_pic, dsize=(arrow_Width, arrow_Height), interpolation=cv2.INTER_AREA)

    gray_arrow = cv2.cvtColor(arrow_pic, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray_arrow, 1, 255, cv2.THRESH_BINARY_INV)

    arrow_roi = image[arrow_Height: Height, (Width/2 - arrow_Width/2) : (Width/2 + arrow_Width/2)]
    arrow_roi = cv2.add(arrow_pic, arrow_roi, mask=mask)
    res = cv2.add(arrow_roi, arrow_pic)
    image[(Height - arrow_Height): Height, (Width/2 - arrow_Width/2): (Width/2 + arrow_Width/2)] = res

    cv2.imshow('steer', image)

######## main ########

if __name__ == '__main__':

    rospy.init_node("line_find")
    bridge = CvBridge()

    pub = rospy.Publisher("error", Int32, queue_size=1)
    rospy.Subscriber("/usb_cam/image_raw/", Image, img_callback)

    msg = Int32()
    msg.data = 0

    while not rospy.is_shutdown():
        if img_from_cam.shape != (Height, Width ,3):
            continue
        image = img_from_cam.copy()
        center = process_image(image)      
        angle = -320 + center

        pub.publish(angle)

        cv2.imshow("cam", image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

