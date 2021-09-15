#!/usr/bin/python3
# -*- coding: UTF-8 -*-
##############跟踪踢球################
import sys
if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)
    
import cv2
import time
import math
import threading
import numpy as np
from LABConfig import *
import hiwonder.PID as pid
from hiwonder import Board
import hiwonder.ActionGroupControl as AGC

debug = False
x_center = 500
x_dis = 500
y_min = 290
y_dis = y_min

x_1 = 680
x_2 = 320
y_2 = y_dis + 300
x_pid = pid.PID(P=0.03, I=0.01, D=0.002)#pid初始化
y_pid = pid.PID(P=0.03, I=0.01, D=0.002)

#初始化时云台舵机的位置
Board.setBusServoPulse(20, y_dis, 500)#控制上下转动，参数2为位置值，范围0-1000，对应0-240度，500为时间，单位毫秒
Board.setBusServoPulse(19, x_dis, 500)#控制左右转动，参数2为位置值，范围0-1000，对应0-240, 500为时间，单位毫秒

#使用到的动作组，存储在/home/pi/AiNexPro/ActionGroups/
go_forward = 'go4'
go_forward_large = 'go4'
go_turn_left = 'go4'
go_turn_right = 'go4'
turn_right = 't_r'
turn_left  = 't_l'        
left_move = 'l_m'
right_move = 'r_m'
left_shot = 'left_shot'
right_shot = 'right_shot'

orgFrame = None
orgframe = None
Running = True
#摄像头默认分辨率640x480,这里做缩小处理
ori_width, ori_height = 320, 240
#调用摄像头
cap = cv2.VideoCapture(-1)
def get_img():
    global orgFrame
    while True:
        if cap.isOpened():
            #读取图片
            ret, orgframe = cap.read()          
            #将摄像头画面缩小以便处理
            orgFrame = cv2.resize(orgframe, (ori_width, ori_height), interpolation = cv2.INTER_CUBIC)
        else:
            time.sleep(0.01)

th1 = threading.Thread(target=get_img)
th1.setDaemon(True)     # 设置为后台线程，这里默认是False，设置为True之后则主线程不用等待子线程
th1.start()  

#映射函数
def leMap(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

#获取最大轮廓以及它的面积
def getAreaMax_contour(contours, area_min = 75):
        contour_area_temp = 0
        contour_area_max = 0
        area_max_contour = None

        for c in contours : #历遍所有轮廓
            contour_area_temp = math.fabs(cv2.contourArea(c)) #计算轮廓面积
            if contour_area_temp > contour_area_max :
                contour_area_max = contour_area_temp
                if contour_area_temp > area_min:  #最大面积的轮廓才是有效的，以过滤干扰
                    area_max_contour = c

        return area_max_contour, contour_area_max#返回最大的轮廓

range_rgb = {'red': (0, 0, 255),
              'blue': (255, 0,0),
              'green': (0, 255, 0),
              'black': (0, 0, 0),
              }

X = 0
Y = 0
ball_status = -1
dis_ok = False
#函数功能：识别特定颜色并且计算出云台舵机跟踪需要转动的值
#参数1：要识别的图像
#参数2：要识别的颜色，默认蓝色，可填值参考/home/pi/AiNexPro/Functions/LABConfig.py
#如果需要识别其他颜色，修改config.py文件，加入其他颜色的字典即可
def color_identify(img, target_color = 'blue'):
    global X, Y    
    global x_dis, y_dis
    global ball_status, dis_ok

    img_center_x = img.shape[:2][1]/2#获取图像宽度值的一半
    img_center_y = img.shape[:2][0]/2
    GaussianBlur_img = cv2.GaussianBlur(img, (3, 3), 0)#高斯模糊
    LAB_img = cv2.cvtColor(GaussianBlur_img, cv2.COLOR_BGR2LAB) #将图像转换到LAB空间
    inRange_img = cv2.inRange(LAB_img, color_range[target_color][0], color_range[target_color][1]) #根据lab值对图片进行二值化 
    opened = cv2.morphologyEx(inRange_img, cv2.MORPH_OPEN, np.ones((3,3),np.uint8))#开运算
    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3,3),np.uint8))#闭运算
    contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2] #找出所有外轮廓
    areaMax_contour = getAreaMax_contour(contours, area_min = 3)[0] #找到最大的轮廓
    
    X = 999
    Y = 999
    radius = 0
    if areaMax_contour is not None:
        if target_color != 'black':
            (X, Y), radius = cv2.minEnclosingCircle(areaMax_contour) #获取最小外接圆的圆心以及半径
            X = int(X)
            Y = int(Y)
            radius = int(radius)
            cv2.circle(orgframe, (X, Y), radius, range_rgb[target_color], 2)#用圆圈框出识别的颜色
            if radius > 2:#半径太小的不做处理
                ########pid控制#########
                #x_pid处理的是控制水平的舵机，y_pid处理控制竖直的舵机
                #以图像的中心点的x，y坐标作为设定的值，以当前识别的颜色中心x，y坐标作为输入
            
                x_pid.SetPoint = img_center_x#设定
                x_pid.update(X)#当前
                x_pwm = x_pid.output#输出
                x_dis += x_pwm
                x_dis = int(x_dis)
                #限制处理，限制水平舵机的旋转范围为0-180,默认的是0-240
                if x_dis < 125:
                    x_dis = 125
                elif x_dis > 875:
                    x_dis = 875                 
                
                y_pid.SetPoint = img_center_y
                y_pid.update(2*img_center_y - Y)#图像的纵像素点从上往下是增大的，而舵机从上往下转动时值减小，故做此处理
                y_pwm = y_pid.output
                y_dis -= y_pwm
                y_dis = int(y_dis)
                #限制处理，防止舵机舵转
                if y_dis < y_min:
                    y_dis = y_min
                elif y_dis > 555:
                    y_dis = 555
                ball_status = 1
                if action_finish:
                    dis_ok = True
                #print(x_dis, y_dis, X, Y)
            else:
                ball_status = 0
    else:
        ball_status = 0
        X, Y = -999, -999
        
action_finish = True
#云台跟踪
def track():
    global dis_ok
    global action_finish
        
    while True:
        if dis_ok is True:
            dis_ok = False
            action_finish = False
            Board.setBusServoPulse(20, y_dis, 20)
            Board.setBusServoPulse(19, x_dis, 20)
            time.sleep(0.02)
            action_finish = True
        else:
            time.sleep(0.01)
     
#作为子线程开启
th2 = threading.Thread(target=track)
th2.setDaemon(True)
th2.start()

step = 1
count = 0
have_go  = False
find_ball = False
find_goal = False
start_turn = False
#机器人跟踪线程
def move():    
    global ball_status
    global X, Y, radius
    global x_dis, y_dis
    global find_ball, start_turn    
    global step, kick, count, have_go
    global go_straight, turn_left, turn_right, go_straight_kick
   
    count_turn = 0
    status = 1
    turn_time = 500
    delay_time = 0.5
    while True:       
        if ball_status == 1:
            count = 0
            if step == 1:
                if y_dis != y_min:#我们在pid控制时限制y_dis最小为900，所以当为900时云台转到了最下面
                    if 125 <= x_dis < x_center - 60:#不在中心，500为中心
                        if y_dis >= 320:
                            AGC.runAction(go_turn_right)
                        else:
                            AGC.runAction(turn_right)
                    elif x_center + 60 < x_dis <= 875:#不在中心
                        if y_dis >= 320:
                            AGC.runAction(go_turn_left)
                        else:
                            AGC.runAction(turn_left)
                    else:#在中心
                        AGC.runAction(go_forward_large)
                else:
                    step = 2
            elif step == 2:#当云台转到了最下，并且找到了球，根据云台的转向来确定球的位置
                if y_dis > y_min or Y <= 160:
                    AGC.runAction(go_forward)
                elif y_dis == y_min and 240 >= Y > 160 and 875 > x_dis > x_center + 120:
                    AGC.runAction(left_move)
                elif y_dis == y_min and 240 >= Y > 160 and x_center - 120 > x_dis > 125:
                    AGC.runAction(right_move)
                elif y_dis == y_min and 240 >= Y > 160 and x_dis > x_center:
                    AGC.runAction(left_shot)
                elif y_dis == y_min and 240 >= Y > 160 and x_dis <= x_center:
                    AGC.runAction(right_shot)
                step = 1
            ball_status = 2
        elif ball_status == 0:#找不到球时，依次向5个方向转动，寻找球
            count += 1
            if count >= 20:
                count = 0
                if start_turn:
                    if count_turn == 0:
                        x_dis = 500
                        y_dis = y_min
                        Board.setBusServoPulse(20, y_dis, turn_time)
                        Board.setBusServoPulse(19, x_dis, turn_time)
                        time.sleep(delay_time)
                    AGC.runAction(turn_left)
                    count_turn += 1
                    if count_turn >= 3:
                        start_turn = False
                        count_turn = 0
                    continue 
                if status == 1:
                    status = 2
                    x_dis = x_1#pid的输出值要跟着更新
                    y_dis = y_min
                    Board.setBusServoPulse(20, y_dis, turn_time)
                    Board.setBusServoPulse(19, x_dis, turn_time)
                    time.sleep(delay_time)
                elif status == 2:
                    status = 3
                    x_dis = x_1
                    y_dis = y_2
                    Board.setBusServoPulse(20, y_dis, turn_time)
                    Board.setBusServoPulse(19, x_dis, turn_time)
                    time.sleep(delay_time)
                elif status == 3:    
                    status = 4
                    x_dis = 500
                    y_dis = y_2
                    Board.setBusServoPulse(20, y_dis, turn_time)
                    Board.setBusServoPulse(19, x_dis, turn_time)
                    time.sleep(delay_time)
                elif status == 4:
                    status = 5
                    x_dis = x_2
                    y_dis = y_2
                    Board.setBusServoPulse(20, y_dis, turn_time)
                    Board.setBusServoPulse(19, x_dis, turn_time)
                    time.sleep(delay_time)
                elif status == 5:
                    status = 1
                    start_turn = True
                    x_dis = x_2
                    y_dis = y_min
                    Board.setBusServoPulse(20, y_dis, turn_time)
                    Board.setBusServoPulse(19, x_dis, turn_time)
                    time.sleep(delay_time)
        else:
            time.sleep(0.01)

#初始化立正
AGC.runAction('1')    
#作为子线程开启
th3 = threading.Thread(target=move)
th3.setDaemon(True)
th3.start()

while True:
    if orgFrame is not None:
        if Running:
            #对原图像不做任何处理，只作为显示，防止出错
            orgframe = orgFrame.copy()
            frame = orgFrame.copy()
            frame_width  = frame.shape[:2][1]
            frame_height = frame.shape[:2][0]
            color_identify(frame, target_color = 'blue')                
            if debug == 1:#调试模式下
                cv2.namedWindow('orgframe', cv2.WINDOW_AUTOSIZE)#显示框命名
                cv2.imshow('orgframe', orgframe) #显示图像
                cv2.waitKey(1)
        else:
            time.sleep(0.01)
    else:
        time.sleep(0.01)
cv2.destroyAllWindows()
