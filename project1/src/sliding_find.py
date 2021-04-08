#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge



######## 고정 변수 ########

Width, Height = 640, 480
mtx = np.array([[422.037858, 0.0, 245.895397], 
                [0.0, 435.589734, 163.625535], 
                [0.0, 0.0, 1.0]])
dist = np.array([-0.289296, 0.061035, 0.001786, 0.015238, 0.0])




######## 설정 변수 ########

img_from_cam = np.empty(shape=[0])

# camera calibration 설정
calibrated = True

# warping  설정
warp_img_w, warp_img_h = 320, 180
warpx_mid, warpx_margin_hi, warpx_margin_lo, warpy_hi, warpy_lo, tilt = 330, 90, 320, 280, 400, 0
if calibrated: warpx_mid, warpx_margin_hi, warpx_margin_lo, warpy_hi, warpy_lo, tilt = 300, 180, 320, 330, 430, 0

# 이진화 설정
hls_threshold = 120

# sliding window 설정
nwindows = 10
window_threshold = 25
margin = 80



######## 유도 변수 ########

cal_mtx, cal_roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (Width, Height), 1, (Width, Height))

warp_src  = np.array([[warpx_mid+tilt-warpx_margin_hi, warpy_hi], [warpx_mid+tilt+warpx_margin_hi, warpy_hi], 
                      [warpx_mid-warpx_margin_lo,  warpy_lo], [warpx_mid+warpx_margin_lo, warpy_lo]], dtype=np.float32)

warp_dist = np.array([[0,0], [warp_img_w,0],
                      [0,warp_img_h], [warp_img_w, warp_img_h]], dtype=np.float32)

M = cv2.getPerspectiveTransform(warp_src, warp_dist)

Minv = cv2.getPerspectiveTransform(warp_src, warp_dist)

midpoint = warp_img_w / 2

window_height  = warp_img_h / nwindows




######## 함수 ########

# 어안렌즈 칼리브레이션
def calibrate_image(frame):
    tf_image = cv2.undistort(frame, mtx, dist, None, cal_mtx)
    x, y, w, h = cal_roi
    tf_image = tf_image[y:y+h, x:x+w]
    return cv2.resize(tf_image, (Width, Height))

# 원근법 적용
def warp_perspective(img):
    warp_img = cv2.warpPerspective(img, M, (warp_img_w, warp_img_h), flags=cv2.INTER_LINEAR)
    cv2.imshow("bird-eye-view", warp_img)
    return warp_img

# 이진화
def convert2binary(img):
    blur = cv2.GaussianBlur(img,(5, 5), 0)
    hls = cv2.cvtColor(blur, cv2.COLOR_BGR2HLS)
    _, L, _ = cv2.split(hls)
    _, binary = cv2.threshold(L, hls_threshold, 255, cv2.THRESH_BINARY)
    cv2.imshow("binary", binary)
    return binary

# 슬라이딩 윈도우 기법으로 차선 찾기 및 이차함수 근사
def find_lane(binary):

    splits = np.vsplit(binary, 10)

    leftx_current, rightx_current = warp_img_w / 4, 3* warp_img_w / 4
    left_miss, right_miss = 2, 2

    lx, ly, rx, ry = [], [], [], []

    out_img = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)

    for i, split in enumerate(splits[::-1]):
        
        # 윈도우 범위 설정
        l_win_x1, l_win_x2 = max(0, leftx_current - margin), min(warp_img_w, leftx_current + margin)
        r_win_x1, r_win_x2 = max(0, rightx_current - margin), min(warp_img_w, rightx_current + margin)

        # 윈도우 생성
        l_win = split[:, l_win_x1 : l_win_x2]
        r_win = split[:, r_win_x1 : r_win_x2]

        # 왼쪽 윈도우에 흰점이 일정 개수 이상이면 차선으로 간주, 중앙값을 대표값으로 설정
        if left_miss > 0 and np.count_nonzero(l_win) > window_threshold :
            leftx_current = l_win_x1 + int(np.median(l_win.nonzero()[1]))
            #cv2.rectangle(out_img,(l_win_x1, warp_img_h -i * window_height),(l_win_x2, warp_img_h - (i+1) * window_height), (0,255,0), 2)
            lx.append(leftx_current)
            ly.append(warp_img_h - i * window_height - window_height/2)
            cv2.circle(out_img, (leftx_current , warp_img_h -i * window_height - window_height/2), 5, (0,255,0), cv2.FILLED)
        else :
            left_miss -= 1

        # 오른쪽 윈도우에 흰점이 일정 개수 이상이면 차선으로 간주, 중앙값을 대표값으로 설정
        if right_miss > 0 and np.count_nonzero(r_win) > window_threshold :
            rightx_current = r_win_x1 + int(np.median(r_win.nonzero()[1]))
            #cv2.rectangle(out_img,(r_win_x1, warp_img_h -i * window_height),(r_win_x2, warp_img_h - (i+1) * window_height), (0,0,255), 2)
            rx.append(rightx_current)
            ry.append(warp_img_h -i * window_height - window_height/2)
            cv2.circle(out_img, (rightx_current , warp_img_h -i * window_height - window_height/2), 5, (0,0,255), cv2.FILLED)
        else :
            right_miss -= 1

    # 이차함수 근사
    lfit = np.polyfit(np.array(ly), np.array(lx), 2) if len(lx) >= 3 else [0,0,0]
    rfit = np.polyfit(np.array(ry), np.array(rx), 2) if len(rx) >= 3 else [0,0,320]
    cv2.imshow("viewer", out_img)

    return lfit, rfit

def img_callback(msg):
    global img_from_cam
    img_from_cam = bridge.imgmsg_to_cv2(msg, "bgr8")

def main():
    while not rospy.is_shutdown():
        frame = img_from_cam.copy()
        if frame.shape != (480,640,3):
            continue
        if calibrated : frame = calibrate_image(frame)
        warp_img = warp_perspective(frame)
        binary = convert2binary(warp_img)
        left_fit, right_fit = find_lane(binary)
        error = -160 + int((left_fit[2]+right_fit[2])/2)
        is_straight = int(max(abs(left_fit[0]), abs(right_fit[0])) < 0.0005)
        msg.data = [error, is_straight]
        pub.publish(msg)

        cv2.imshow('camera', frame)

        if cv2.waitKey(33) == ord('q') :
            break

    cv2.destroyAllWindows()


if __name__ == '__main__':
    rospy.init_node("line_find")
    bridge = CvBridge()

    pub = rospy.Publisher("error", Int32MultiArray, queue_size=1)
    rospy.Subscriber("/usb_cam/image_raw/", Image, img_callback)

    msg = Int32MultiArray()
    msg.data = [0,1]
    main()

