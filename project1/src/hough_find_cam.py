#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import cv2, random, math
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge


######## 고정값 ########

# 영상 크기
Width, Height  = 640, 480

# 색상 코드
red, green, blue, yellow = (0, 0, 255), (0, 255, 0), (255, 0, 0), (0, 255, 255)

# 이미지 초기값
img_from_cam = np.array([])

error_prev = 0


######## 중앙선 영상처리 설정값 ########

# ROI 영역 설정
m_offset_y, m_gap_y, m_offset_x, m_gap_x = 280, 50, 290, 60

# HoughLineP 설정
hough_threshold_m, min_length_m, min_gap_m = 5, 15, 20

# 차선 기울기 필터링 범위
slope_threshold = 0.5




######## 양쪽 차선 영상처리 설정값 ########

# ROI 영역 설정
Offset, Gap = 340, 40

# HoughLineP 설정
hough_threshold, min_length, min_gap = 30, 30, 10

# 군집 임계값
cluster_threshold = 30

# 기울기 임계값
slope_lim = 0.15

# 수평선 위치
horizon_line = 280
converge_threshold = 100

# gaussianBlur 설정
kernel_size = (5,5)

# Canny 설정
low_threshold, high_threshold = 60, 70



######## 함수 ########

# 콜백 함수
def img_callback(data):
    global img_from_cam
    img_from_cam = bridge.imgmsg_to_cv2(data, "bgr8")

# 직사각형 그리기 함수
def draw_rectangle(img, lpos, rpos, offset=0):
    center = (lpos + rpos) / 2
    cv2.rectangle(img, (lpos - 5, 15 + offset), (lpos + 5, 25 + offset), green, 2)
    cv2.rectangle(img, (rpos - 5, 15 + offset), (rpos + 5, 25 + offset), green, 2)
    cv2.rectangle(img, (center-5, 15 + offset), (center+5, 25 + offset), green, 2)    
    cv2.rectangle(img, (315, 15 + offset), (325, 25 + offset), red, 2)
    cv2.rectangle(img, (0, Offset), (Width - 1, Offset + Gap), red, 2)
    cv2.rectangle(img, (m_offset_x, m_offset_y), (m_offset_x+m_gap_x, m_offset_y+m_gap_y), yellow, 2)

# 선 군집 함수
def get_cluster(lines, frame):
    cluster = []
    for x1, y1, x2, y2 in lines[:, 0]:
        if x1 == x2 or abs(float(y2-y1)/(x2-x1)) > slope_lim:
            cv2.line(frame, (x1, y1+Offset), (x2, y2+Offset), red, 2)
            x_top = (0-y1)*(x2-x1)/(y2-y1) + x1
            x_bottom = (Gap-y1)*(x2-x1)/(y2-y1) + x1
            for lines in cluster:
                if abs(lines[0][0] - x_top) < cluster_threshold and abs(lines[0][1] - x_bottom) < cluster_threshold:
                    lines.append([x_top, x_bottom])
                    break
            else:
                cluster.append([[x_top, x_bottom]])
    return cluster

# 에러값 계산 함수
def get_error(cluster, frame):
    if not cluster:
        return error_prev
    converge_points = []
    for lines in cluster:
        mean_x_top, mean_x_bottom = np.mean(lines, 0)
        cv2.line(frame, (int(mean_x_top), Offset), (int(mean_x_bottom), Offset+Gap-1), green, 3)
        x_converge = (horizon_line-Offset)*(mean_x_bottom-mean_x_top)/(Gap) + mean_x_top
        converge_points.append(x_converge)
    x_converge_mean = int(np.mean(converge_points))
    cv2.circle(frame, (x_converge_mean, horizon_line), 5, (255,255,0), cv2.FILLED)
    return x_converge_mean -320

# 양쪽 차선 영상처리 과정
def process_roi(frame):

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, kernel_size, 0)
    canny = cv2.Canny(blur, low_threshold, high_threshold)
    roi = canny[Offset : Offset+Gap, 0 : Width]

    lines = cv2.HoughLinesP(roi, 1, math.pi/180, hough_threshold, min_length, min_gap)
    if lines is not None:
        cluster = get_cluster(lines, frame)
        error = get_error(cluster, frame)
    else:
        error = None
    return error

# 중앙선 영상처리 과정
def process_m_roi(img):
    roi = img[m_offset_y : m_offset_y + m_gap_y, m_offset_x : m_offset_x + m_gap_x]
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, 180, 255, cv2.THRESH_BINARY)
    lines = cv2.HoughLinesP(binary, 1, math.pi/180, hough_threshold_m, None, min_length_m, min_gap_m)
    is_straight = 0
    if lines is not None:
        is_straight =  get_state(img, lines)
    return is_straight

# 직선/커브 판별 함수
def get_state(img, lines):
    is_straight = False
    for x1, y1, x2, y2 in lines[:, 0]:
        m = float("inf") if y1 == y2 else float(x2-x1) / float(y2-y1)
        if abs(m) < slope_threshold :
            is_straight = True
            cv2.line(img, (x1+m_offset_x, y1+m_offset_y), (x2+m_offset_x, y2+m_offset_y), yellow, 2)
    return is_straight




######## main ########

if __name__ == '__main__':

    rospy.init_node("line_find")
    bridge = CvBridge()

    pub = rospy.Publisher("error", Int32MultiArray, queue_size=1)
    rospy.Subscriber("/usb_cam/image_raw/", Image, img_callback)

    msg = Int32MultiArray()
    msg.data = [0,1]

    while not rospy.is_shutdown():
        if img_from_cam.shape != (Height, Width ,3):
            continue
        
        image = img_from_cam.copy()

        # 중앙선 영상 처리
        is_straight = process_m_roi(image)

        # 양쪽 차선 영상 처리
        error = process_roi(image)      
        if error is None:
            error = error_prev
        error_prev = error
        # 결과값 발행
        msg.data = [error,is_straight]
        pub.publish(msg)

        cv2.imshow("roi", image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

