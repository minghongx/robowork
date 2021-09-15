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

APRILTAG_SIZE = 100 # tag大小mm
FACTOR = 105*400/APRILTAG_SIZE #当距离tag400mm时w的大小为68
DISTANCE_TO_CAMERA = 120 #mm

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
head_turn = 'left_right'
object_center_x, object_center_y, object_angle, object_distance = -2, -2, 0, 0

# 变量重置
def reset():
    global time_start
    global d_x, d_y
    global start_count
    global step, head_turn
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
    head_turn = 'left_right'
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
            if contour_area_temp >= 100:  # 只有在面积大于设定时，最大面积的轮廓才是有效的，以过滤干扰
                area_max_contour = c
                max_area = contour_area_temp

    return area_max_contour, max_area  # 返回最大的轮廓, 面积

# 红绿蓝颜色识别
size = (320, 240)
def colorDetect(img):
    img_h, img_w = img.shape[:2]

    frame_resize = cv2.resize(img, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间

    center_max_distance = pow(img_w / 2, 2) + pow(img_h, 2)
    color, center_x, center_y, angle = 'None', -1, -1, 0
    for i in color_range:
        if i in color_list:
            frame_mask = cv2.inRange(frame_lab, color_range[i][0], color_range[i][1])  # 对原图像和掩模进行位运算
            eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 腐蚀
            dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 膨胀
            contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓
            areaMaxContour, area_max = getAreaMaxContour(contours)  # 找出最大轮廓

            if area_max > 1000:  # 有找到最大面积
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

# 检测apriltag
detector = apriltag.Detector(searchpath=apriltag._get_demo_searchpath())
def apriltagDetect(img):
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    detections = detector.detect(gray, return_image=False)
    tag_1 = [-1, -1, 0, 0]
    tag_2 = [-1, -1, 0, 0]
    tag_3 = [-1, -1, 0, 0]

    if len(detections) != 0:
        for detection in detections:
            pose, e0, e1 = detector.detection_pose(detection,
                                                (fx, fy, cx, cy),
                                                    1.0)
            '''
            apriltag._draw_pose(img,
                                (fx, fy, cx, cy),
                                 1.0,
                                 pose)
            '''
            sy = math.sqrt(pow(pose[0][0], 2) + pow(pose[1][0], 2))
            if sy >= 1e-6:
                pitch = math.degrees(math.atan2(pose[2][1], pose[2][2]))
                roll = math.degrees(math.atan2(-pose[2][0], sy))
                yaw = math.degrees(math.atan2(pose[1][0], pose[0][0]))
            else:
                pitch = math.degrees(math.atan2(-pose[1][2], pose[1][1]))
                roll = math.degrees(math.atan2(-pose[2][0], sy))
                yaw = 0
            pitch = int(pitch)
            roll = round(roll, 1)
            yaw = int(yaw)
            
            #cv2.putText(img, "pitch:" + str(pitch), (10, img.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 2)
            #cv2.putText(img, "roll:" + str(roll), (10, img.shape[0] - 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 2)
            #cv2.putText(img, "yaw:" + str(yaw), (10, img.shape[0] - 70), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 2)            
            
            corners = np.rint(detection.corners)  # 获取四个角点
            cv2.drawContours(img, [np.array(corners, np.int)], -1, (0, 255, 255), 2)

            w = math.sqrt(pow(corners[0][0] - corners[1][0], 2) + pow(corners[0][1] - corners[1][1], 2))
            h = math.sqrt(pow(corners[0][0] - corners[3][0], 2) + pow(corners[0][1] - corners[3][1], 2))

            tag_family = str(detection.tag_family, encoding='utf-8')  # 获取tag_family
            tag_id = str(detection.tag_id)  # 获取tag_id

            object_center_x, object_center_y = int(detection.center[0]), int(detection.center[1])  # 中心点
            cv2.circle(frame, (object_center_x, object_center_y), 5, (0, 255, 255), -1)
            object_angle = roll
            
            if tag_family == 'tag36h11':
                distance = int(APRILTAG_SIZE*FACTOR/w)
                if tag_id == '1':
                    tag_1 = [object_center_x, object_center_y, object_angle, distance]
                elif tag_id == '2':
                    tag_2 = [object_center_x, object_center_y, object_angle, distance]
                elif tag_id == '3':
                    tag_3 = [object_center_x, object_center_y, object_angle, distance]

    return tag_1, tag_2, tag_3

# 通过其他apriltag判断目标apriltag位置
# apriltag摆放位置：红(tag36h11_1)，绿(tag36h11_2)，蓝(tag36h11_3)
def getTurn(tag_id, tag_data):
    tag_1 = tag_data[0]
    tag_2 = tag_data[1]
    tag_3 = tag_data[2]

    if tag_id == 1:  # 目标apriltag为1
        if tag_2[0] == -1:  # 没有检测到apriltag 2
            if tag_3[0] != -1:  # 检测到apriltag 3， 则apriltag 1在apriltag 3左边，所以左转
                return 'left'
        else:  # 检测到apriltag 2，则则apriltag 1在apriltag 2左边，所以左转
            return 'left'
    elif tag_id == 2:
        if tag_1[0] == -1:
            if tag_3[0] != -1:
                return 'left'
        else:
            return 'right'
    elif tag_id == 3:
        if tag_1[0] == -1:
            if tag_2[0] != -1:
                return 'right'
        else:
            return 'right'

    return 'None'

turn = 'None'
state = ''
CENTER_X = 305
amplitude = 20
find_box = True
stop_detect = False
object_color = 'red'
head_turn = 'left_right'
color_list = ['red', 'green', 'blue']
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
    global head_turn
    global time_start
    global color_list
    global stop_detect
    global current_pos
    global amplitude
    global object_center_x
    
    while True:
        if __isRunning:
            if object_center_x == -3:  # -3表示放置阶段， 且没有找到目标apriltag，但是找到其他apriltag
                # 根据其他arpiltag的相对位置来判断转向
                if turn == 'left':
                    if state != 'l':
                        ik.stand(current_pos)
                    state = 'l'
                    ik.turn_left(current_pos, mode, 10, 150, 1)
                elif turn == 'right':
                    if state != 'r':
                        ik.stand(current_pos)
                    state = 'r'
                    ik.turn_right(current_pos, mode, 10, 150, 1)
            elif object_center_x >= 0:  # 如果找到目标
                if step == 1:  # 左右调整，保持在正中
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
                        if object_center_x - CENTER_X < -150:  # 不在中心，根据方向让机器人转向一步
                            if state != 'l_m':
                                ik.stand(current_pos)
                            state = 'l_m'
                            ik.left_move(current_pos, mode, 30, 150, 1)
                        elif object_center_x - CENTER_X > 150:
                            if state != 'r_m':
                                ik.stand(current_pos)
                            state = 'r_m'
                            ik.right_move(current_pos, mode, 30, 150, 1)
                        elif object_angle < -10:
                            if state != 'l':
                                ik.stand(current_pos)
                            state = 'l'
                            ik.turn_right(current_pos, mode, 5, 150, 1)
                        elif 10 < object_angle:
                            if state != 'r':
                                ik.stand(current_pos)
                            state = 'r'
                            ik.turn_left(current_pos, mode, 5, 150, 1)
                        elif object_center_x - CENTER_X > 100:  # 不在中心，根据方向让机器人转向一步
                            if state != 'r_m':
                                ik.stand(current_pos)
                            state = 'r_m'
                            ik.right_move(current_pos, mode, 30, 150, 1)
                        elif object_center_x - CENTER_X < -100:
                            if state != 'l_m':
                                ik.stand(current_pos)
                            state = 'l_m'
                            ik.left_move(current_pos, mode, 30, 150, 1)
                        else:
                            step = 2
                elif step == 2:  # 接近物体
                    if find_box:
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
                    else:
                        if object_distance < 280:
                            if state != 'b':
                                ik.stand(current_pos)
                            state = 'b'
                            ik.back(current_pos, mode, 80, 150, 1)
                            step = 1
                        elif 340 < object_distance:
                            if state != 'g':
                                ik.stand(current_pos)
                            state = 'g'
                            ik.go_forward(current_pos, mode, 100, 150, 1)
                            step = 1
                        else:
                            step = 3
                elif step == 3:  # 再次左右调整
                    if find_box:
                        if 100 <= object_center_x - CENTER_X:  # 不在中心，根据方向让机器人转向一步
                            if state != 'r_m':
                                ik.stand(current_pos)
                            state = 'r_m'
                            ik.right_move(current_pos, mode, 20, 80, 1)
                        elif object_center_x - CENTER_X < -100:
                            if state != 'l_m':
                                ik.stand(current_pos)
                            state = 'l_m'
                            ik.left_move(current_pos, mode, 20, 80, 1)
                        elif 40 <= object_center_x - CENTER_X:  # 不在中心，根据方向让机器人转向一步
                            if state != 'r':
                                ik.stand(current_pos)
                            state = 'r'
                            ik.turn_right(current_pos, mode, 5, 80, 1)
                        elif object_center_x - CENTER_X < -40:
                            if state != 'l':
                                ik.stand(current_pos)
                            state = 'l'
                            ik.turn_left(current_pos, mode, 5, 80, 1)
                        else:
                            step = 4
                    else:
                        if object_angle > 5:
                            if state != 'l':
                                ik.stand(current_pos)
                            state = 'l'
                            ik.turn_left(current_pos, mode, 5, 150, 1)
                        elif -5 > object_angle:
                            if state != 'r':
                                ik.stand(current_pos)
                            state = 'r'
                            ik.turn_right(current_pos, mode, 5, 150, 1)
                        elif object_center_x - CENTER_X > 50:  # 不在中心，根据方向让机器人转向一步
                            if state != 'r_m':
                                ik.stand(current_pos)
                            state = 'r_m'
                            ik.right_move(current_pos, mode, 30, 150, 1)
                        elif object_center_x - CENTER_X < -50:
                            if state != 'l_m':
                                ik.stand(current_pos)
                            state = 'l_m'
                            ik.left_move(current_pos, mode, 30, 150, 1)
                        else:
                            step = 4
                elif step == 4:  # 靠近物体
                    if find_box:
                        if object_center_y < 200:
                            ik.go_forward(current_pos, mode, 20, 80, 1)
                        else:
                            if abs(object_center_x - CENTER_X) <= 40:
                                stop_detect = True
                                step = 5
                            else:
                                step = 3
                    else:
                        if object_distance > 340:
                            ik.go_forward(current_pos, mode, 60, 150, 1)
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
                        Board.setPWMServoPulse(1, 1500, 500)
                        mode = 4
                        find_box = not find_box
                        object_center_x = -2
                        step = 1
                        stop_detect = False
                    else:
                        current_pos = copy.deepcopy(ik.initial_pos)
                        Pick.pick(False)
                        mode = 2
                        Board.setPWMServoPulse(1, servo1, 500)
                        ik.back(current_pos, mode, 50, 100, 3)                       
                        color_list.remove(object_color)
                        if color_list == []:
                            color_list = ['red', 'green', 'blue']
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
                        ik.turn_right(current_pos, mode, 15, 100, 1)  # 原地右转
                        #time.sleep(0.5)
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
    global turn
    global stop_detect, find_box
    global color, color_center_x, color_center_y, color_angle
    global object_color, object_center_x, object_center_y, object_angle, object_distance

    if not __isRunning or stop_detect:
        if step == 5:
            object_center_x = 0

        # img = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)

        return img

    # 如果是搬运阶段
    if find_box:
        color, color_center_x, color_center_y, color_angle = colorDetect(img)  # 颜色检测，返回颜色，中心坐标，角度
        object_color, object_center_x, object_center_y, object_angle = color, color_center_x, color_center_y, color_angle
    else:
        tag_data = apriltagDetect(img)  # apriltag检测
        #print(tag_data)
        if tag_data[color_tag[object_color] - 1][0] != -1:  # 如果检测到目标arpiltag
            object_center_x, object_center_y, object_angle, object_distance = tag_data[color_tag[object_color] - 1]
        else:  # 如果没有检测到目标arpiltag，就通过其他arpiltag来判断相对位置
            turn = getTurn(color_tag[object_color], tag_data)
            if turn == 'None':
                object_center_x, object_center_y, object_angle, object_distance = -1, -1, 0, 0
            else:  # 完全没有检测到apriltag
                object_center_x, object_center_y, object_angle, object_distance = -3, -1, 0, 0

    # img = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)  # 畸变矫正

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
