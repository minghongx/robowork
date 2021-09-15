#!/usr/bin/python3
# coding=utf8
import time
from time import sleep
from HiwonderPuppy import PUPPY, BusServoParams
import numpy as np

import sys
sys.path.append('/home/pi/PuppyPi_PC_Software/')

from ServoCmd import *


if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)


puppy = PUPPY(setServoPulse = setBusServoPulse, servoParams = BusServoParams())


def stance(x = 0, y = 0, z = -15, x_shift = 2):# 单位cm
    # x_shift越小，走路越前倾，越大越后仰,通过调节x_shift可以调节小狗走路的平衡
    return np.array([
                        [x + x_shift, x + x_shift, -x + x_shift, -x + x_shift],
                        [y, y, y, y],
                        [z, z, z, z],
                    ])#此array的组合方式不要去改变


puppy.stance_config(stance(0,0,-15,2), pitch = 0, roll = 0)# 标准站姿
puppy.gait_config(overlap_time = 0.1, swing_time = 0.15, z_clearance = 3)
# overlap_time:4脚全部着地的时间，swing_time：2脚离地时间，z_clearance：走路时，脚抬高的距离

puppy.run() # 启动
# puppy.move(x=5, y=0, yaw_rate = 0)
# 行走的x，y速度（cm/s）和旋转（转弯）速度（rad/s），都为0，即原地踏步
puppy.move_stop()

# puppy.stance_config(stance(0,0,15,1))
# puppy.stance_config(stance(0,0,15,2))
# puppy.stance_config(stance=stance(0,0,13,-4), pitch = -20/57.3, roll = 0/57.3)
# puppy.stance_config(stance(0,0,12,1))
# puppy.gait_config(overlap_time = 0.4, swing_time = 0.6, z_clearance = 2)
# puppy.stance_config(stance=stance(0,0,13,4), pitch = 20/57.3, roll = 0)

# sleep(5)
# puppy.move_stop()
# print(puppy.get_coord()) # 获取当前4个腿的坐标

# runActionGroup('coord_up_stair_2')



##################################################################
import cv2
import math
import threading
sys.path.append('/home/pi/PuppyPi/')
from HiwonderSDK.Board import *
import HiwonderSDK.Board as Board
import Camera
from LABConfig import *


SERVO_VALUE_LOOK_DOWN = 2450
SERVO_VALUE_LOOK_UP = 1500
SERVO_VALUE_DEVIATION =30


def setServoPulse(id, pulse, use_time):
    setPWMServoPulse(id, pulse, use_time)


__target_color = ('black',)
# 设置检测颜色
def setLineTargetColor(target_color):
    global __target_color

    __target_color = target_color
    return (True, ())

# 初始位置 
def initMove():
    Board.setPWMServoPulse(1, SERVO_VALUE_LOOK_DOWN + SERVO_VALUE_DEVIATION, 500)  

line_center = -2
last_line_center = 0

find_stairs = False
find_gate = False
find_cross = False
# 变量重置
def reset():
    global last_line_center
    global line_center
    global __target_color
    global find_stairs
    global find_gate
    global find_cross

    last_line_center = 0
    line_center = -2
    __target_color = ()
    find_stairs = False
    find_gate = False
    find_cross = False
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


def _run():
    roi_black_line = (240, 280,  0, 640)
    roi_gate = (160, 320,  0, 640)
    start_find_gate = False
    have_found_gate = False
    complete_through_gate = False
    last_time = time.time()
    last_time_find_cross = last_time

    start_find_cross = False
    img_gate_standard = cv2.imread('gate_standard.jpg')
    img_gate_standard = cv2.cvtColor(img_gate_standard, cv2.COLOR_BGR2GRAY)
    img_cross_standard = cv2.imread('cross_standard.png')
    img_cross_standard = cv2.cvtColor(img_cross_standard, cv2.COLOR_BGR2GRAY)

    # img_cross_standard_contour, _ = getAreaMaxContour(cv2.findContours(img_cross_standard, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2])
    time_wait = 0

    img_center_x = 320
    find_stairs_times = 0
    def move():
        global last_line_center
        global line_center
        
        nonlocal img_center_x
        nonlocal find_stairs_times


        if __isRunning:
            time.sleep(0.003)
            # print(last_line_center,line_center)
            if find_cross:
                puppy.move_stop()
                print('puppy.move_stop()')
                return

            if find_stairs:
                if find_stairs_times == 0:
                    puppy.move(x=5, y=0, yaw_rate = 0)
                    puppy.move_stop()
                    sleep(0.2)
                    runActionGroup('coord_up_stair_1')
                    sleep(9)
                    puppy.stance_config(stance=stance(0,0,-13,-4), pitch = -20/57.3, roll = 0)
                    puppy.move_stop()
                    sleep(0.2)
                    puppy.gait_config(overlap_time = 0.4, swing_time = 0.6, z_clearance = 2)
                    puppy.move(x=2, y=0, yaw_rate = 0)
                    sleep(5)
                    puppy.move_stop()
                    sleep(0.2)
                    runActionGroup('coord_up_stair_2')
                    sleep(7)

                    puppy.stance_config(stance=stance(0,0,-13,2), pitch = 0, roll = 0)
                    puppy.gait_config(overlap_time = 0.2, swing_time = 0.3, z_clearance = 2)
                    puppy.move_stop()
                    sleep(0.2)
                    puppy.move(x=5, y=0, yaw_rate = 0)
                    sleep(3)

                    find_stairs_times += 1
                elif find_stairs_times == 1:
                    puppy.move(x=5, y=0, yaw_rate = 0)
                    sleep(1.9)
                    puppy.move_stop()
                    sleep(0.5)
                    runActionGroup('coord_down_stair')
                    sleep(6)

                    puppy.stance_config(stance=stance(0,0,-13,4), pitch = 20/57.3, roll = 0)
                    puppy.gait_config(overlap_time = 0.2, swing_time = 0.3, z_clearance = 2)
                    puppy.move_stop()
                    sleep(0.2)
                    puppy.move(x=2, y=0, yaw_rate = 0)
                    sleep(7)
                    
                    puppy.stance_config(stance=stance(0,0,-15,2), pitch = 0, roll = 0)
                    puppy.gait_config(overlap_time = 0.2, swing_time = 0.3, z_clearance = 2)
                    puppy.move_stop()
                    sleep(0.2)
                    puppy.move(x=2, y=0, yaw_rate = 0)
                    sleep(2)

                    puppy.stance_config(stance=stance(0,0,-15,2))
                    puppy.gait_config(overlap_time = 0.1, swing_time = 0.15, z_clearance = 3)
                    puppy.move_stop()
                    sleep(0.2)

                    find_stairs_times += 1

            elif line_center >= 0:              
                if abs(line_center -img_center_x) < 60:
                    # go_forward
                    if find_stairs_times == 1:
                        puppy.move(x=5, y=0, yaw_rate = 0)
                    else:
                        puppy.move(x=10, y=0, yaw_rate = 0)
                elif line_center -img_center_x >= 60:
                    # turn_right
                    puppy.move(x=5, y=0, yaw_rate = -25/57.3)
                else:
                    # turn_left
                    puppy.move(x=5, y=0, yaw_rate = 25/57.3)
                last_line_center = line_center

            elif line_center == -1:
                # turn_left
                if last_line_center >= img_center_x:
                    puppy.move(x=5, y=0, yaw_rate = 25/57.3)
                else:
                    # turn_right
                    puppy.move(x=5, y=0, yaw_rate = -25/57.3)
        else:
            pass
            # time.sleep(0.01)

    def fun(img):
        global line_center
        global __target_color
        global find_stairs
        global find_gate
        global find_cross

        nonlocal img_center_x
        nonlocal roi_black_line
        nonlocal roi_gate
        nonlocal start_find_gate
        nonlocal have_found_gate
        nonlocal complete_through_gate
        nonlocal last_time
        nonlocal last_time_find_cross
        nonlocal time_wait
        nonlocal start_find_cross
        # nonlocal img_cross_standard_contour
        nonlocal find_stairs_times

        if not __isRunning or __target_color == ():
            return img

        color_aggregate = ('black','yellow')

        # test
        # color_aggregate = ('blue',)
        # Board.setPWMServoPulse(1, SERVO_VALUE_LOOK_UP + SERVO_VALUE_DEVIATION, 100)

        if complete_through_gate == False:
            if find_gate:
                if time.time() - last_time >= time_wait + 8:
                    # 8秒通过矮门
                    puppy.stance_config(stance=stance(0,0,-15,1))
                    complete_through_gate = True
                elif time.time() - last_time >= time_wait:
                    puppy.stance_config(stance=stance(0,0,-12,1))
            else:
                if start_find_gate == False:
                    if line_center >= 0 and abs(line_center -img_center_x) < 60:  
                        if time.time() - last_time >= 0.9:# 连续直行超过这个时间，才开始抬头（也就是弯道不要抬头）
                            last_time = time.time()            
                            Board.setPWMServoPulse(1, SERVO_VALUE_LOOK_UP + SERVO_VALUE_DEVIATION, 100)
                            start_find_gate = True
                    else:
                        last_time = time.time()

                if start_find_gate:
                    if time.time() - last_time >= 0.3:#确定舵机转动到指定角度
                        color_aggregate = ('blue',)
                    else:
                        # print('return')
                        return

        if start_find_cross:
            if time.time() - last_time_find_cross> 4:
                # 发现十字起始线，并且在指定时间后才停止，确保狗子通过起始线
                puppy.move(x=0, y=0, yaw_rate = 0)
                puppy.move_stop()
                find_cross = True
                return

        # test
        # color_aggregate = ('black',)
        # Board.setPWMServoPulse(1, SERVO_VALUE_LOOK_DOWN + SERVO_VALUE_DEVIATION, 100)

        frame_gb = cv2.GaussianBlur(img, (3, 3), 3)
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间

        for color in color_aggregate:
            frame_mask = cv2.inRange(frame_lab, color_range[color][0], color_range[color][1])  # 对原图像和掩模进行位运算
            
            
            # cnts = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]  #找出所有轮廓
            # cnt_large, area = getAreaMaxContour(cnts)  #找到最大面积的轮廓
            # print(color,'dilated')

            if color == 'blue': # 矮门
                # dilated = dilated[roi_gate[0]:roi_gate[1], roi_gate[2]:roi_gate[3]]
                eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (10, 10)))  #腐蚀
                dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (10, 10)))  #膨胀
                cnts = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]  #找出所有轮廓
                cnt_large, area = getAreaMaxContour(cnts)  #找到最大面积的轮廓
                x,y,w,h = cv2.boundingRect(cnt_large)


                # dist = cv2.matchShapes(img_gate_standard_contour, cnt_large, cv2.CONTOURS_MATCH_I1, 0)
                dist = cv2.matchShapes(img_gate_standard, dilated[y:y+h,x:x+w], cv2.CONTOURS_MATCH_I1, 0)
                # <0.003差不多认定是矮门，如果混入杂质，小于0.006，0.008也可
                
                
                #test
                img2 = dilated.copy()
                img2 = cv2.cvtColor(img2,cv2.COLOR_GRAY2BGR)
                img2 = cv2.putText(img2, 'w*h=%d area=%d dist=%.5f ' % (w*h,area,dist,), (0, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                # img2 = cv2.putText(img2, str(area), (0, 80), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 2)
                cv2.rectangle(img2, (x, y), (x+w, y+h), (255, 0, 0), 3)
                # cv2.imshow('blue',img2)


                # print('dist',dist)
                # print('gate_area',w*h)
                # print('w',w)
                # print('centre', x+w//2,y+h//2)

                # if dilated[y:y+h,x:x+w].size != 0:
                #     img = cv2.putText(img, str(dist), (0, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 0, 0), 2)
                #     cv2.imwrite('img'+ str(time.time()) + '.png',img)
                #     cv2.imwrite('dilated'+ str(time.time()) + '.png',dilated[y:y+h,x:x+w])
                # cv2.waitKey(0)


                if (dist < 0.01 and w*h > 29000) or w*h > 190000 or area > 75000:
                    find_gate = True
                    Board.setBuzzer(1)
                    Board.setPWMServoPulse(1, SERVO_VALUE_LOOK_DOWN + SERVO_VALUE_DEVIATION, 100)
                    sleep(0.3)
                    Board.setBuzzer(0)
                    

                    if (dist < 0.01 and w*h > 29000):
                        time_wait = 13.3 - w*6/290
                    elif w*h > 190000:
                        time_wait = 0.5
                    elif area > 75000:
                        time_wait = 0

                    # w*h=29000差不多是1米的距离,144500是0.4米的距离
                    # 1m:w=256  0.4m:w=546
                    # 摄像头距离矮门的距离cm = 153 - w*6/29
                    # 以10cm每秒的速度前进，距离20cm开始调低身体
                    # (153 - gate_w*6/29 - 20)/10


                    # print('dist',dist)
                    # print('gate_area',w*h)
                    # print('area',area)
                    # print('w',w)
                    # print('time_wait',time_wait)
                    img2 = cv2.putText(img2, 'time_wait=%.5f ' % time_wait, (0, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                    cv2.imwrite('img2_'+ str(time.time()) + '.png',img2)

                    # rect = cv2.minAreaRect(cnt_large)  #最小外接矩形
                    # box = np.int0(cv2.boxPoints(rect))  #最小外接矩形的四个顶点
                    # cv2.drawContours(img, [box], -1, (0, 255, 255), 2)  #画出四个点组成的矩形
                    # cv2.imshow('img', img)

                    last_time = time.time()
                else:
                    start_find_gate = False
                    Board.setPWMServoPulse(1, SERVO_VALUE_LOOK_DOWN + SERVO_VALUE_DEVIATION, 100)
                    sleep(0.3)
                    cv2.imwrite('img2_'+ str(time.time()) + '.png',img2)
                    last_time = time.time() 

            if color == 'black': # 黑线
                eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (20, 20)))  #腐蚀
                dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (10, 10)))  #膨胀


                # if time.time() - last_time_find_cross > 5:
                if find_stairs_times >= 2: # 过了台阶后开始识别十字起始线
                    dist = cv2.matchShapes(img_cross_standard, dilated, cv2.CONTOURS_MATCH_I1, 0)
                    if dist < 0.01:# 认为识别到了十字黑线
                        print(dist)
                        cnts = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]  #找出所有轮廓
                        cnt_large, area = getAreaMaxContour(cnts)
                        x,y,w,h = cv2.boundingRect(cnt_large)

                        cv2.imwrite('test_dilated' + str(time.time()) + '.png',dilated)
                        
                        Board.setBuzzer(1)
                        sleep(0.1)
                        Board.setBuzzer(0)

                        print('w*h=',(w*h))
                        if w * h == 640*480: # 
                            start_find_cross = True
                            last_time_find_cross = time.time()

                # # test
                # dist = cv2.matchShapes(img_cross_standard_contour, getAreaMaxContour(cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2])[0], cv2.CONTOURS_MATCH_I1, 0)
                # # cv2.imwrite('cross_standard.png', dilated)
                # # cv2.waitKey()
                # img2 = dilated.copy()
                # img2 = cv2.cvtColor(img2,cv2.COLOR_GRAY2BGR)
                # img2 = cv2.putText(img2, 'dist=%.6f ' % dist, (0, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                # cv2.imshow('img2', img2)


                dilated = dilated[roi_black_line[0]:roi_black_line[1], roi_black_line[2]:roi_black_line[3]]
                cnts = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]  #找出所有轮廓
                cnt_large, area = getAreaMaxContour(cnts)  #找到最大面积的轮廓
                # print(area)
                if area > 1500:
                    rect = cv2.minAreaRect(cnt_large)  #最小外接矩形
                    
                    box = np.int0(cv2.boxPoints(rect))  #最小外接矩形的四个顶点
                    for j in range(4):
                        box[j, 1] = box[j, 1] + roi_black_line[0]

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
                # print('if color == black:',line_center)
            elif color == 'yellow': # 台阶
                eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  #腐蚀
                dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  #膨胀
                cnts = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]  #找出所有轮廓
                cnt_large, area = getAreaMaxContour(cnts)  #找到最大面积的轮廓
                # print(area)
                if area > 20000:
                    rect = cv2.minAreaRect(cnt_large)  #最小外接矩形
                    
                    box = np.int0(cv2.boxPoints(rect))  #最小外接矩形的四个顶点

                    cv2.drawContours(img, [box], -1, (0, 255, 255), 2)  #画出四个点组成的矩形
                    find_stairs = True
                else:
                    find_stairs = False
        move()
        return img
    return fun

run = _run()



if __name__ == '__main__':
# if False:
    import HiwonderSDK.Sonar as Sonar
    # from CameraCalibration.CalibrationConfig import *
    
    # #加载参数
    # param_data = np.load(calibration_param_path + '.npz')

    # #获取参数
    # mtx = param_data['mtx_array']
    # dist = param_data['dist_array']
    # newcameramtx, _ = cv2.getOptimalNewCameraMatrix(mtx, dist, (640, 480), 0, (640, 480))
    # mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (640, 480), 5)

    init()
    start()
    __target_color = ('black','yellow','blue')
    my_camera = Camera.Camera()
    my_camera.camera_open()
    while True:
        img = my_camera.frame
        if img is not None:
            frame = img.copy()
            # frame = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)  # 畸变矫正  
            Frame = run(frame)
            # cv2.imshow('Frame', Frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
            time.sleep(0.00001)
        else:
            time.sleep(0.01)
    my_camera.camera_close()
    cv2.destroyAllWindows()
