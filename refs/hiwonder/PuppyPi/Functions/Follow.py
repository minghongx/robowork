#!/usr/bin/python3
# coding=utf8
import sys
import cv2
import import_path
import time
import math
import Camera
import threading
import numpy as np
from LABConfig import *
import apriltag as apriltag
import kinematics as kinematics
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *

ik = kinematics.IK()

servo1 = 1500
HWSONAR = None

APRILTAG_SIZE = 36 # mm
FACTOR = 147*100/APRILTAG_SIZE
DISTANCE_TO_CAMERA = 120 #/mm

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

#加载参数
param_data = np.load(calibration_param_path + '.npz')

#获取参数
mtx = param_data['mtx_array']
dist = param_data['dist_array']
newcameramtx, _ = cv2.getOptimalNewCameraMatrix(mtx, dist, (640, 480), 0, (640, 480))
mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (640, 480), 5)

fx = mtx[0, 0]
fy = mtx[1, 1]
cx = mtx[0, 2]
cy = mtx[1, 2]

# 初始位置
def initMove():
    Board.setPWMServoPulse(1, servo1, 500)
    Board.setPWMServoPulse(2, servo2, 500)
    ik.stand(ik.initial_pos)

ditance, angle = -1, 0
# 变量重置
def reset():
    global __target_color
    global distance, angle

    __target_color = ()
    distance, angle = -1, 0

# app初始化调用
def init():
    print("Follow Init")
    initMove()

__isRunning = False
# app开始玩法调用
def start():
    global __isRunning
    reset()
    __isRunning = True
    print("Follow Start")

# app停止玩法调用
def stop():
    global __isRunning
    __isRunning = False
    print("Follow Stop")

# app退出玩法调用
def exit():
    global __isRunning
    __isRunning = False
    print("Follow Exit")

#执行动作组
def move():

    while True:
        if __isRunning:
            if distance > 0:
                if angle > 5:
                    if 40 > angle:
                        ik.turn_right(ik.initial_pos, 2, angle, 50, 1)
                    else:
                        time.sleep(0.01)
                elif angle < -5:
                    if -40 < angle:
                        ik.turn_left(ik.initial_pos, 2, -angle, 50, 1)
                    else:
                        time.sleep(0.01)
                elif 250 < distance:
                    d_d = int(distance - 250)
                    if d_d > 150:
                        ik.go_forward(ik.initial_pos, 2, 150, 60, 1)
                    elif d_d > 10:
                        ik.go_forward(ik.initial_pos, 2, d_d, 50, 1)
                    else:
                        time.sleep(0.01)
                elif 0 < distance < 150:
                    ik.back(ik.initial_pos, 2, 150, 100, 1)
                else:
                    time.sleep(0.01)
            else:
                time.sleep(0.01)

#启动动作的线程
th = threading.Thread(target=move)
th.setDaemon(True)
th.start()

# 检测apriltag
detector = apriltag.Detector(searchpath=apriltag._get_demo_searchpath())
def apriltagDetect(img):
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    detections = detector.detect(gray, return_image=False)
    tag = [-1, -1, 0]
    
    if len(detections) != 0:
        for detection in detections:
            corners = detection.corners
            corners = np.rint(corners)  # 获取四个角点
            w = math.sqrt(pow(corners[0][0] - corners[1][0], 2) + pow(corners[0][1] - corners[1][1], 2))
            h = math.sqrt(pow(corners[0][0] - corners[3][0], 2) + pow(corners[0][1] - corners[3][1], 2))
            
            cv2.drawContours(img, [np.array(corners, np.int)], -1, (0, 255, 255), 2)
            
            tag_family = str(detection.tag_family, encoding='utf-8')  # 获取tag_family
            tag_id = str(detection.tag_id)  # 获取tag_id

            object_center_x, object_center_y = int(detection.center[0]), int(detection.center[1])  # 中心点
            cv2.circle(img, (object_center_x, object_center_y), 5, (0, 255, 255), -1)

            if tag_family == 'tag36h11':
                tag = [object_center_x, object_center_y, w]

    return tag

def calculate_rotation(center_x, img_width, apriltag_width):
    x = (center_x - img_width/2)*(APRILTAG_SIZE/apriltag_width)
    distance = int(APRILTAG_SIZE*FACTOR/apriltag_width)
    angle = int(math.degrees(math.atan2(x, DISTANCE_TO_CAMERA + distance)))
    return angle, distance

def run(img):
    global distance, angle
    
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
    
    if not __isRunning:
        return img

    tag = apriltagDetect(img)
    if tag[0] != -1:
        centerX, centerY, apriltag_width = tag[0], tag[1], tag[2]
        angle, distance = calculate_rotation(centerX, img_w, apriltag_width)
        #print(angle, distance)
    else:
        angle, distance  = 0, 0

    return img

if __name__ == '__main__':
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
