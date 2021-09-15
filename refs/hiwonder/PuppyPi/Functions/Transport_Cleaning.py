#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import cv2
import copy
import time
import math
import import_path
import Camera
import apriltag
import threading
import numpy as np
from LABConfig import *
import PickAction as Pick
import kinematics as kinematics
import HiwonderSDK.Misc as Misc
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *

HWSONAR = None

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

ik = kinematics.IK()

# 加载参数
param_data = np.load(calibration_param_path + '.npz')

# 获取参数
mtx = param_data['mtx_array']
dist = param_data['dist_array']
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (640, 480), 0, (640, 480))
mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (640, 480), 5)

fx = mtx[0, 0]
fy = mtx[1, 1]
cx = mtx[0, 2]
cy = mtx[1, 2]

mode = 2
current_pos = copy.deepcopy(ik.initial_pos)

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

color_tag = {'red': 1,
             'green': 2,
             'blue': 3
             }

__target_color = ('red',)

# 设置检测颜色
def setBallTargetColor(target_color):
    global __target_color

    __target_color = target_color
    return (True, ())

# 初始位置
def initMove():
    HWSONAR.setRGBMode(0)
    HWSONAR.setRGB(1, (0, 0, 0))
    HWSONAR.setRGB(2, (0, 0, 0))    
    Board.setPWMServoPulse(1, servo1, 500)
    Board.setPWMServoPulse(2, servo2, 500)  
    ik.stand(current_pos)

d_x = 15
d_y = 15
step = 1
time_start = 0
x_dis = servo2
y_dis = servo1
start_count = True
__isRunning = False
object_center_x, object_center_y, object_angle, object_distance = -2, -2, 0, 0

# 变量重置
def reset():
    global time_start
    global d_x, d_y
    global start_count
    global step
    global x_dis, y_dis
    global __target_color
    global object_center_x, object_center_y, object_angle, object_distance

    d_x = 15
    d_y = 15
    step = 1
    time_start = 0
    x_dis = servo2
    y_dis = servo1
    start_count = True
    __target_color = ()
    HWSONAR.setRGBMode(0)
    HWSONAR.setRGB(1, (0, 0, 0))
    HWSONAR.setRGB(2, (0, 0, 0))
    object_center_x, object_center_y, object_angle, object_distance = -2, -2, 0, 0

# app初始化调用
def init():
    print("Self_Transport Init")
    initMove()

# app开始玩法调用
def start():
    global __isRunning
    reset()
    __isRunning = True
    print("Self_Transport Start")

# app停止玩法调用
def stop():
    global __isRunning
    __isRunning = False
    print("Self_Transport Stop")

# app退出玩法调用
def exit():
    global __isRunning
    __isRunning = False
    ik.stand(ik.initial_pos)
    print("Self_Transport Exit")

# 找出面积最大的轮廓
# 参数为要比较的轮廓的列表
def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0

    area_max_contour = None
    max_area = 0

    for c in contours:  # 历遍所有轮廓
        contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp >= 10:  # 只有在面积大于设定时，最大面积的轮廓才是有效的，以过滤干扰
                area_max_contour = c
                max_area = contour_area_temp

    return area_max_contour, max_area  # 返回最大的轮廓, 面积

# 红绿蓝颜色识别
size = (320, 240)
def colorDetect(img, color_list):
    img_h, img_w = img.shape[:2]

    frame_resize = cv2.resize(img, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间

    center_max_distance = pow(img_w / 2, 2) + pow(img_h, 2)
    color, center_x, center_y, angle = 'None', -1, -1, 0
    for i in color_range:
        if i in color_list:
            frame_mask = cv2.inRange(frame_lab, color_range[i][0], color_range[i][1])  # 对原图像和掩模进行位运算
            if i == 'black':
                frame_mask[:, :100] = 0
                frame_mask[:, 220:] = 0
            eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 腐蚀
            dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 膨胀
            contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓
            areaMaxContour, area_max = getAreaMaxContour(contours)  # 找出最大轮廓
            
            if area_max > 1000 or (i == 'black' and area_max > 0):  # 有找到最大面积
                rect = cv2.minAreaRect(areaMaxContour)  # 最小外接矩形
                angle_ = rect[2]

                box = np.int0(cv2.boxPoints(rect))  # 最小外接矩形的四个顶点
                for j in range(4):
                    box[j, 0] = int(Misc.map(box[j, 0], 0, size[0], 0, img_w))
                    box[j, 1] = int(Misc.map(box[j, 1], 0, size[1], 0, img_h))

                cv2.drawContours(img, [box], -1, (0, 255, 255), 2)  # 画出四个点组成的矩形

                # 获取矩形的对角点
                ptime_start_x, ptime_start_y = box[0, 0], box[0, 1]
                pt3_x, pt3_y = box[2, 0], box[2, 1]
                center_x_, center_y_ = int((ptime_start_x + pt3_x) / 2), int((ptime_start_y + pt3_y) / 2)  # 中心点
                cv2.circle(img, (center_x_, center_y_), 5, (0, 255, 255), -1)  # 画出中心点

                distance = pow(center_x_ - img_w / 2, 2) + pow(center_y_ - img_h, 2)
                if distance < center_max_distance:  # 寻找距离最近的物体来搬运
                    center_max_distance = distance
                    color = i
                    center_x, center_y, angle = center_x_, center_y_, angle_

    return color, center_x, center_y, angle

turn = 'None'
state = ''
CENTER_X = 320
find_box = True
stop_detect = False
object_color = 'red'
red_color, red_center_x, red_center_y = 'None', -1, -1
color_center_x, color_center_y = -1, -1
# 执行动作组
def move():
    global d_x
    global d_y
    global mode
    global step
    global x_dis
    global y_dis
    global state
    global start_count
    global find_box
    global time_start
    global stop_detect
    global current_pos
    global object_center_x
    
    while True:
        if __isRunning:
            if object_center_x >= 0:  # 如果找到目标
                if find_box and red_center_y > object_center_y and object_center_y > 0:
                    ik.turn_right(current_pos, mode, 15, 100, 1)
                elif step == 1:  # 左右调整，保持在正中
                    if find_box:
                        if object_center_x - CENTER_X > 160:  # 不在中心，根据方向让机器人转向一步
                            ik.right_move(current_pos, mode, 50, 80, 1)
                        elif object_center_x - CENTER_X < -160:
                            ik.left_move(current_pos, mode, 50, 80, 1)
                        elif -10 > object_angle > -45:
                            ik.turn_left(current_pos, mode, 15, 80, 1)
                        elif -80 < object_angle <= -45:
                            ik.turn_right(current_pos, mode, 15, 80, 1)
                        elif object_center_x - CENTER_X > 80:  # 不在中心，根据方向让机器人转向一步
                            ik.right_move(current_pos, mode, 20, 80, 1)
                        elif object_center_x - CENTER_X < -80:
                            ik.left_move(current_pos, mode, 20, 80, 1)
                        else:
                            step = 2
                    else:
                        if object_center_y < 300:
                            ik.go_forward(current_pos, mode, 80, 150, 1)
                        else:
                            step = 5
                elif step == 2:  # 接近物体
                    if 240 < object_center_y:
                        ik.back(current_pos, mode, 50, 80, 1)
                        step = 1
                    elif 100 < object_center_y < 150:
                        ik.go_forward(current_pos, mode, 40, 80, 1)
                        step = 1
                    elif 0 < object_center_y <= 100:
                        ik.go_forward(current_pos, mode, 80, 100, 1)
                        step = 1
                    else:
                        step = 3
                elif step == 3:  # 再次左右调整
                    if 100 <= object_center_x - CENTER_X:  # 不在中心，根据方向让机器人转向一步
                        ik.right_move(current_pos, mode, 20, 80, 1)
                    elif object_center_x - CENTER_X < -100:
                        ik.left_move(current_pos, mode, 20, 80, 1)
                    elif 40 <= object_center_x - CENTER_X:  # 不在中心，根据方向让机器人转向一步
                        ik.turn_right(current_pos, mode, 5, 80, 1)
                    elif object_center_x - CENTER_X < -40:
                        ik.turn_left(current_pos, mode, 5, 80, 1)
                    else:
                        step = 4
                elif step == 4:  # 靠近物体
                    if find_box:
                        if object_center_y < 210:
                            ik.go_forward(current_pos, mode, 15, 80, 1)
                        else:
                            if abs(object_center_x - CENTER_X) <= 40:
                                stop_detect = True
                                step = 5
                            else:
                                step = 3
                elif step == 5:  # 拿起或者放下物体
                    if find_box:
                        current_pos = copy.deepcopy(ik.initial_pos_quadruped)
                        Pick.pick(True)
                        mode = 4
                        find_box = not find_box
                        object_center_x = -2
                        step = 1
                        stop_detect = False
                    else:
                        ik.go_forward(current_pos, mode, 40, 150, 2)
                        current_pos = copy.deepcopy(ik.initial_pos)
                        Pick.pick(False)
                        mode = 2
                        ik.back(current_pos, mode, 50, 100, 3)                       
                        find_box = not find_box
                        object_center_x = -2
                        step = 1
                        stop_detect = False
            elif object_center_x == -1:  # 找不到目标时，转头，转身子来寻找
                if start_count:
                    start_count = False
                    time_start = time.time()
                else:
                    if time.time() - time_start > 2:
                        if find_box:
                            ik.turn_right(current_pos, mode, 15, 100, 1)  # 原地右转
                        else:
                            ik.go_forward(current_pos, mode, 80, 150, 1)
            else:
                time.sleep(0.01)
        else:
            time.sleep(0.01)

# 启动动作的线程
th = threading.Thread(target=move)
th.setDaemon(True)
th.start()

def run(img):
    global step
    global stop_detect, find_box
    global red_color, red_center_x, red_center_y
    global object_color, object_center_x, object_center_y

    if not __isRunning or stop_detect:
        if step == 5:
            object_center_x = 0

        return img
    
    red_color, red_center_x, red_center_y, red_angle = colorDetect(img, ['red'])  # 颜色检测，返回颜色，中心坐标，角度
    # 如果是搬运阶段
    if find_box:
        color, color_center_x, color_center_y, color_angle = colorDetect(img, ['green', 'blue'])  # 颜色检测，返回颜色，中心坐标，角度
        object_color, object_center_x, object_center_y, object_angle = color, color_center_x, color_center_y, color_angle
    else:
        object_color, object_center_x, object_center_y, object_angle = red_color, red_center_x, red_center_y, red_angle

    return img

if __name__ == '__main__':
    import HiwonderSDK.Sonar as Sonar

    HWSONAR = Sonar.Sonar()
    init()
    start()
    my_camera = Camera.Camera()
    my_camera.camera_open()
    while True:
        img = my_camera.frame
        if img is not None:
            frame = img.copy()
            Frame = run(frame)
            cv2.imshow('Frame', Frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
        else:
            time.sleep(0.01)
    my_camera.camera_close()
    cv2.destroyAllWindows()
