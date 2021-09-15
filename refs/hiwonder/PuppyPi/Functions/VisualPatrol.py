#!/usr/bin/python3
# coding=utf8
import sys
import cv2
import time
import import_path
import math
import Camera
import threading
import numpy as np
from LABConfig import *
import kinematics as kinematics
import HiwonderSDK.Board as Board

ik = kinematics.IK()
HWSONAR = None

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

__target_color = ('red',)
# 设置检测颜色
def setLineTargetColor(target_color):
    global __target_color

    __target_color = target_color
    return (True, ())

# 初始位置
def initMove():
    HWSONAR.setRGBMode(0)
    HWSONAR.setRGB(1, (0, 0, 0))
    HWSONAR.setRGB(2, (0, 0, 0))
    Board.setPWMServoPulse(1, 1000, 500)
    Board.setPWMServoPulse(2, servo2, 500)    

line_center = -2
last_line_center = 0
# 变量重置
def reset():
    global last_line_center
    global line_center
    global __target_color

    last_line_center = 0
    line_center = -2
    __target_color = ()
    
# app初始化调用
def init():
    print("VisualPatrol Init")
    initMove()

__isRunning = False
# app开始玩法调用
def start():
    global __isRunning
    reset()
    __isRunning = True
    print("VisualPatrol Start")

# app停止玩法调用
def stop():
    global __isRunning
    __isRunning = False
    print("VisualPatrol Stop")

# app退出玩法调用
def exit():
    global __isRunning
    __isRunning = False
    ik.stand(ik.initial_pos)
    print("VisualPatrol Exit")

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

    return area_max_contour, max_area  # 返回最大的轮廓

img_center_x = 320
def move():
    global last_line_center
    global line_center

    while True:
        if __isRunning:
            if line_center >= 0:              
                if abs(line_center -img_center_x) < 60:
                    ik.go_forward(ik.initial_pos, 2, 30, 50, 1)
                elif line_center -img_center_x >= 60:
                    ik.turn_right(ik.initial_pos, 2, 10, 50, 1)
                else:
                    ik.turn_left(ik.initial_pos, 2, 10, 50, 1)
                last_line_center = line_center

            elif line_center == -1:
                if last_line_center >= img_center_x:
                    ik.turn_left(ik.initial_pos, 2, 10, 50, 1)
                else:
                    ik.turn_right(ik.initial_pos, 2, 10, 50, 1)
        else:
            time.sleep(0.01)

# 运行子线程
th = threading.Thread(target=move)
th.setDaemon(True)
th.start()

roi = [(240, 280,  0, 640)]
def run(img):
    global line_center
    global __target_color

    if not __isRunning or __target_color == ():
        return img

    frame_gb = cv2.GaussianBlur(img, (3, 3), 3)
    
    for r in roi:
        blobs = frame_gb[r[0]:r[1], r[2]:r[3]]
        frame_lab = cv2.cvtColor(blobs, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间

        for i in color_range:
            if i in __target_color:
                detect_color = i
                frame_mask = cv2.inRange(frame_lab, color_range[detect_color][0], color_range[detect_color][1])  # 对原图像和掩模进行位运算
                eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  #腐蚀
                dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  #膨胀
                cnts = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]  #找出所有轮廓
                cnt_large, area = getAreaMaxContour(cnts)  #找到最大面积的轮廓
                if area > 10:
                    rect = cv2.minAreaRect(cnt_large)  #最小外接矩形
                    
                    box = np.int0(cv2.boxPoints(rect))  #最小外接矩形的四个顶点
                    for j in range(4):
                        box[j, 1] = box[j, 1] + r[0]

                    cv2.drawContours(img, [box], -1, (0, 255, 255), 2)  #画出四个点组成的矩形

                    #获取矩形的对角点
                    pt1_x, pt1_y = box[0, 0], box[0, 1]
                    pt3_x, pt3_y = box[2, 0], box[2, 1]
                    line_center_x, line_center_y = (pt1_x + pt3_x) / 2, (pt1_y + pt3_y) / 2  #中心点
                    cv2.circle(img, (int(line_center_x), int(line_center_y)), 5, (0, 0, 255), -1)  #画出中心点
                    line_center = line_center_x
                else:
                    if line_center != -2:
                        line_center = -1
        
    return img

if __name__ == '__main__':
    import HiwonderSDK.Sonar as Sonar
    from CameraCalibration.CalibrationConfig import *
    
    #加载参数
    param_data = np.load(calibration_param_path + '.npz')

    #获取参数
    mtx = param_data['mtx_array']
    dist = param_data['dist_array']
    newcameramtx, _ = cv2.getOptimalNewCameraMatrix(mtx, dist, (640, 480), 0, (640, 480))
    mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (640, 480), 5)

    HWSONAR = Sonar.Sonar()
    init()
    start()
    __target_color = ('red',)
    my_camera = Camera.Camera()
    my_camera.camera_open()
    while True:
        img = my_camera.frame
        if img is not None:
            frame = img.copy()
            frame = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)  # 畸变矫正
            Frame = run(frame)           
            cv2.imshow('Frame', Frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
        else:
            time.sleep(0.01)
    my_camera.camera_close()
    cv2.destroyAllWindows()
