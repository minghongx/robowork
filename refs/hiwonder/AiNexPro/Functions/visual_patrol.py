#!/usr/bin/python3
# coding=utf8
import sys
import cv2
import time
import math
import Camera
import apriltag
import threading
import numpy as np
from LABConfig import *
from hiwonder import Misc
from hiwonder import Board
import hiwonder.ActionGroupControl as AGC

debug_mode = 1

servo1 = 500
servo2 = 200

__isRunning = True
__line_color = 'black'

stand = 'stand'
stand_after_go = 'stand_after_go'
ladder_up = 'ladder_up'
ladder_down = 'ladder_down'
go_forward = 'go_forward4'
go_turn_left = 'go_turn_left1'
go_turn_right = 'go_turn_right1'
go_forward_one_step = 'go_forward_one_step'
turn_left = 'turn_left_0413'
turn_right = 'turn_right_0413'

range_rgb = {'red': (0, 0, 255),
              'blue': (255, 0,0),
              'green': (0, 255, 0),
              'black': (0, 0, 0),
              'yellow':(255, 255, 0),
              }

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

# 找出面积最大的轮廓
# 参数为要比较的轮廓的列表
def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None

    for c in contours:  # 历遍所有轮廓
        contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp >= 5:  # 只有在面积大于300时，最大面积的轮廓才是有效的，以过滤干扰
                area_max_contour = c

    return area_max_contour, contour_area_max  # 返回最大的轮廓

# 检测apriltag
detector = apriltag.Detector(searchpath=apriltag._get_demo_searchpath())
apriltag_x, apriltag_y, apriltag_angle = -1, -1, None
def apriltagDetect(img):   
    global apriltag_x, apriltag_y, apriltag_angle
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    detections = detector.detect(gray, return_image=False)
    if len(detections) != 0:
        for detection in detections:                       
            corners = np.rint(detection.corners)  # 获取四个角点
            cv2.drawContours(img, [np.array(corners, np.int)], -1, (0, 255, 255), 2)

            tag_family = str(detection.tag_family, encoding='utf-8')  # 获取tag_family
            tag_id = int(detection.tag_id)  # 获取tag_id

            apriltag_x, apriltag_y = int(detection.center[0]), int(detection.center[1])  # 中心点
            
            apriltag_angle = int(math.degrees(math.atan2(corners[0][1] - corners[1][1], corners[0][0] - corners[1][0])))  # 计算旋转角
            
            return tag_family, tag_id
            
    return None, None

block_x, block_y, block_angle = -1, -1, 0
def colorDetect(img, target):
    global block_x, block_y, block_angle
    img_h, img_w = img.shape[:2]

    frame_resize = cv2.resize(img, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间

    center_max_distance = pow(img_w / 2, 2) + pow(img_h, 2)
    center_x, center_y, area_max, angle = -1, -1, 0, 0
    frame_mask = cv2.inRange(frame_lab, color_range[target][0], color_range[target][1])  # 对原图像和掩模进行位运算
    eroded = cv2.erode(frame_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 腐蚀
    dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 膨胀
    contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓
    area_max_contour, area_max = getAreaMaxContour(contours)  # 找出最大轮廓

    if area_max > 500:  # 有找到最大面积
        rect = cv2.minAreaRect(area_max_contour)  # 最小外接矩形
        angle_ = rect[2]

        box = np.int0(cv2.boxPoints(rect))  # 最小外接矩形的四个顶点
        for j in range(4):
            box[j, 0] = int(Misc.map(box[j, 0], 0, size[0], 0, img_w))
            box[j, 1] = int(Misc.map(box[j, 1], 0, size[1], 0, img_h))

        cv2.drawContours(img, [box], -1, (0, 255, 255), 2)  # 画出四个点组成的矩形

        # 获取矩形的对角点
        pt1_x, pt3_x = box[0, 0], box[2, 0]
        center_x_ = int((pt1_x + pt3_x) / 2)
        center_y_ = min(box[0, 1], box[1, 1], box[2, 1], box[3, 1])
        cv2.circle(img, (center_x_, center_y_), 5, (0, 255, 255), -1)  # 画出中心点

        center_x, center_y, angle = center_x_, center_y_, angle_
        block_x, block_y, block_angle = center_x, center_y, angle
    return center_x, center_y, angle

#识别特定的颜色，根据输入的颜色来识别
def color_identify(img, target_color = 'green'):    
    frame_gaussianblur = cv2.GaussianBlur(img, (3, 3), 0)#高斯模糊
    frame_lab = cv2.cvtColor(frame_gaussianblur, cv2.COLOR_BGR2LAB) #将图像转换到LAB空间
    mask = cv2.inRange(frame_lab, color_range[target_color][0], color_range[target_color][1]) #根据lab值对图片进行二值化
    opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3,3),np.uint8))#开运算
    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3,3),np.uint8))#闭运算
    contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2] #找出所有外轮廓
    areaMax_contour, area_max = getAreaMaxContour(contours) #找到最大的轮廓
    
    X = -1
    Y = -1
    if areaMax_contour is not None:  #有找到最大面积
        rect = cv2.minAreaRect(areaMax_contour)#最小外接矩形
        box = np.int0(cv2.boxPoints(rect))#最小外接矩形的四个顶点
        cv2.drawContours(img, [box], -1, (255, 0, 0), 2)#画出四个点组成的矩形
        
        #获取矩形的对角点
        pt1_x, pt1_y = box[0, 0], box[0, 1]
        pt3_x, pt3_y = box[2, 0], box[2, 1]            
        X, Y = (pt1_x + pt3_x) / 2, (pt1_y + pt3_y) / 2#中心点       
        cv2.circle(img, (int(X), int(Y)), 5, (255, 0, 0), -1)#画出中心点
        
        #print('Y',Y, area_max) 
        if area_max > 5000:       
            return X, Y
        else:
            return -1, -1
    else:
        return -1, -1

roi = [ # [ROI, weight]
        (40, 80,  0, 640, 0.1), 
        (140, 180,  0, 640, 0.2), 
        (240, 280,  0, 640, 0.7)
       ]

roi_h1 = roi[0][0]
roi_h2 = roi[1][0] - roi[0][0]
roi_h3 = roi[2][0] - roi[1][0]

roi_h_list = [roi_h1, roi_h2, roi_h3]

y_index = 0
count_cross = 0
size = (640, 480)
start_detect_banner = False
def lineDetect(img):
    global y_index
    global count_go
    global start_count
    global count_cross
    global detect_line
    global detect_tree
    global line_centerx
    global __line_color
    global start_detect_tree
    global start_detect_banner
    
    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
    
    if not __isRunning or __line_color == ():
        return img
    
    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)   
            
    centroid_x_sum = 0
    weight_sum = 0
    center_ = []
    n = 0
    
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间
    frame_mask = cv2.inRange(frame_lab, color_range[__line_color][0], color_range[__line_color][1])  # 对原图像和掩模进行位运算
    opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))  # 开运算
    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))  # 闭运算   
    
    #cv2.imshow('closed', closed)

    #将图像分割成上中下三个部分，这样处理速度会更快，更精确
    y_acc = np.sum(closed[200:280, 0:440], axis=1)
    y_index = np.argmax(y_acc)
    
    #print('tree_number', tree_number) 
    #print(y_index, y_acc[y_index]) 
    
    if tree_number == 0:
        if y_index > 0 and y_acc[y_index] > 20000 and not detect_block and not detect_apriltag and not detect_ladder:
            count_cross += 1
            if count_cross > 5:
                start_detect_tree = True
                detect_line = False
                return
        else:
            count_cross = 0
    else:
        #print('count_go', count_go, y_index)
        if count_go > 25 and y_index > 0 and y_acc[y_index] > 20000 and not detect_block and not detect_apriltag and not detect_ladder:            
            count_cross += 1
            if count_cross > 5:
                if tree_number < 2:
                    start_detect_tree = True
                detect_line = False
                if tree_number == 2:
                    detect_tree = False
                    start_detect_tree = False
                    start_detect_banner = True
                return
        else:
            count_cross = 0  
    for r in roi:
        roi_h = roi_h_list[n]
        n += 1       
        blobs = closed[r[0]:r[1], r[2]:r[3]]
        cnts = cv2.findContours(blobs , cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]#找出所有轮廓
        cnt_large, area = getAreaMaxContour(cnts)#找到最大面积的轮廓
        if cnt_large is not None:#如果轮廓不为空
            rect = cv2.minAreaRect(cnt_large)#最小外接矩形
            box = np.int0(cv2.boxPoints(rect))#最小外接矩形的四个顶点
            for i in range(4):
                box[i, 1] = box[i, 1] + (n - 1)*roi_h + roi[0][0]
                box[i, 1] = int(Misc.map(box[i, 1], 0, size[1], 0, img_h))
            for i in range(4):                
                box[i, 0] = int(Misc.map(box[i, 0], 0, size[0], 0, img_w))
                
            cv2.drawContours(img, [box], -1, (0,0,255,255), 2)#画出四个点组成的矩形
            
            #获取矩形的对角点
            pt1_x, pt1_y = box[0, 0], box[0, 1]
            pt3_x, pt3_y = box[2, 0], box[2, 1]            
            center_x, center_y = (pt1_x + pt3_x) / 2, (pt1_y + pt3_y) / 2#中心点       
            cv2.circle(img, (int(center_x), int(center_y)), 5, (0,0,255), -1)#画出中心点
            
            center_.append([center_x, center_y])                        
            #按权重不同对上中下三个中心点进行求和
            centroid_x_sum += center_x * r[4]
            weight_sum += r[4]

    if weight_sum is not 0:
        #求最终得到的中心点
        cv2.circle(img, (line_centerx, int(center_y)), 10, (0,255,255), -1)#画出中心点
        line_centerx = int(centroid_x_sum / weight_sum)  
    else:
        line_centerx = -1

count = 0
count_go = 0

up = True
start_count = False

banner_y = 0
yellow_y = 0

tree_number = 0
banner_color = 'None'

detect_line = True
detect_tree = False
detect_block = True
detect_apriltag = False
detect_banner = False
detect_ladder = False

have_detect_banner = True
have_detect_ladder = False
have_detect_apriltag = False
have_detect_block = False
banner_color_list = ['red', 'green', 'blue']

line_centerx = -1
img_centerx = 320
have_detect_tree = False
start_detect_tree = False
detect_tree_finish = False
def move():
    global up
    global count_go
    global start_count
    global tree_number      
    global detect_tree
    global detect_line
    global line_centerx
    global detect_banner
    global detect_ladder
    global detect_block
    global detect_apriltag
    global have_detect_tree
    global start_detect_tree
    global start_detect_banner
    global detect_tree_finish
    global have_detect_banner
    global have_detect_ladder
    
    while True:
        if __isRunning:
            print('start_detect_tree', start_detect_tree, detect_tree)
            if not detect_line and start_detect_tree:
                AGC.runAction(stand_after_go)
                Board.setBusServoPulse(19, servo1 - 300, 500)
                Board.setBusServoPulse(20, servo2 + 100, 500)
                time.sleep(0.5)
                Board.setBusServoPulse(19, servo1 - 300, 500)
                Board.setBusServoPulse(20, servo2 + 100, 500)
                time.sleep(0.5)
                detect_tree = True
                start_detect_tree = False
            elif detect_tree_finish:
                if have_detect_tree:
                    tree_number += 1
                    Board.setBuzzer(0)
                    Board.setBuzzer(1)
                    time.sleep(1)
                    Board.setBuzzer(0)
                Board.setBusServoPulse(19, servo1, 500)
                Board.setBusServoPulse(20, servo2, 500)
                time.sleep(0.5)
                Board.setBusServoPulse(19, servo1, 500)
                Board.setBusServoPulse(20, servo2, 500)
                time.sleep(0.5)
                AGC.runAction(go_forward)
                AGC.runAction(go_forward)                
                have_detect_tree = False
                detect_tree_finish = False                
                detect_line = True
                if tree_number == 2:
                    start_count = False
                start_count = True
                detect_tree = False
                count_go = 0
            elif have_detect_ladder and yellow_y > 290:
                detect_line = False
                have_detect_ladder = False
                if up:
                    AGC.runAction(stand_after_go)
                    time.sleep(1)
                    AGC.runAction(ladder_up)
                    up = False
                else:
                    AGC.runAction(go_forward_one_step)
                    #AGC.runAction(go_forward_one_step)
                    time.sleep(1)
                    AGC.runAction(ladder_down)
                    detect_ladder = False
                detect_line = True
            elif start_detect_banner and not detect_line:
                AGC.runAction(stand_after_go)
                Board.setBusServoPulse(19, servo1 + 300, 500)
                Board.setBusServoPulse(20, servo2 + 100, 500)
                time.sleep(0.5)
                Board.setBusServoPulse(19, servo1 + 300, 500)
                Board.setBusServoPulse(20, servo2 + 100, 500)
                time.sleep(0.5)
                start_detect_banner = False
                detect_banner = True
            elif have_detect_banner and banner_y > 400:
                Board.setBusServoPulse(19, servo1, 500)
                Board.setBusServoPulse(20, servo2, 500)
                time.sleep(0.5)     
                Board.setBusServoPulse(19, servo1, 500)
                Board.setBusServoPulse(20, servo2, 500)
                time.sleep(0.5)   
                detect_line = False               
                if banner_color == 'red':
                    AGC.runAction('salute')
                else:
                    AGC.runAction('salute')
                AGC.runAction(go_forward)
                AGC.runAction(go_forward)
                AGC.runAction(go_forward)
                detect_line = True
                detect_banner = False
                have_detect_banner = False
            elif detect_block and have_detect_block:
                if block_x - img_centerx > 50:
                    AGC.runAction(turn_right)
                elif block_x - img_centerx < -50:
                    AGC.runAction(turn_left)
                elif block_y < 10:
                    AGC.runAction(go_forward_one_step)
                else:
                    AGC.runAction('move_up')
                    AGC.runAction('turn_right_with_block')
                    AGC.runAction('turn_right_with_block')
                    AGC.runAction('turn_right_with_block')
                    AGC.runAction('turn_right_with_block')
                    AGC.runAction('turn_right_with_block')
                    AGC.runAction('turn_right_with_block')
                    AGC.runAction('turn_right_with_block')
                    for i in range(4):
                        AGC.runAction('go_forward_with_block_l')
                    detect_block = False
                    detect_apriltag = True
                    detect_line = False                
            elif detect_apriltag:
                if have_detect_apriltag:
                    #print(apriltag_x , apriltag_y)
                    if apriltag_x - img_centerx > 50:
                        AGC.runAction('turn_right_with_block')
                    elif apriltag_x - img_centerx < -50:
                        AGC.runAction('turn_left_with_block')
                    elif apriltag_y < 450:
                        AGC.runAction('go_forward_with_block_l_one')
                    AGC.runAction('put_down')
                    for j in range(12):
                        AGC.runAction(turn_left)
                    for i in range(10):
                        AGC.runAction(go_forward)
                    AGC.runAction(turn_right)
                    AGC.runAction(turn_right)
                    AGC.runAction(turn_right)
                    detect_apriltag = False
                    detect_line = True
                    detect_ladder = True
            elif detect_line:
                if not up and 150 < yellow_y < 290:
                    if abs(line_centerx - img_centerx) <= 50:
                        AGC.runAction(go_forward_one_step)
                    elif line_centerx - img_centerx > 50:
                        AGC.runAction(turn_right)
                    elif line_centerx - img_centerx < -50:
                        AGC.runAction(turn_left)
                elif line_centerx != -1:
                    if start_count:
                        count_go += 1
                    if abs(line_centerx - img_centerx) <= 50:
                        AGC.runAction(go_forward)
                    elif line_centerx - img_centerx > 50:
                        AGC.runAction(go_turn_right)
                    elif line_centerx - img_centerx < -50:
                        AGC.runAction(go_turn_left)
                else:
                    time.sleep(0.01)
        else:
            time.sleep(0.01)

# 运行子线程
th = threading.Thread(target=move)
th.setDaemon(True)
if debug_mode <= 1:
    th.start()

def run(img):
    global up
    global count
    global banner_y   
    global yellow_y
    global have_detect_tree
    global detect_tree_finish
    global have_detect_banner
    global have_detect_ladder
    global have_detect_apriltag
    global have_detect_block
    
    if detect_line:
        lineDetect(img)
    if detect_block:
        block_y = colorDetect(img, 'red')[1]
        if block_y != -1:
            if block_y > 5 and not have_detect_block:
                AGC.stopAction()
                have_detect_block = True
    if detect_apriltag:
#         Board.setBusServoPulse(20, servo2 + 100, 500)
#         time.sleep(0.5)
        a, b = apriltagDetect(img)
        if a is not None and b is not None:
            have_detect_apriltag = True       
    if detect_ladder:
        yellow_y = color_identify(img, 'yellow')[1]
        #print('yellow_y', yellow_y)
        if yellow_y != -1:
            have_detect_ladder = True
            if yellow_y > 270 and not have_detect_ladder:
                AGC.stopAction()            
    if detect_banner:
        for i in banner_color_list:
            banner_y = color_identify(img, i)[1]        
            if banner_y != -1:
                banner_color = i
                have_detect_banner = True        
    if detect_tree and not detect_line:
        if not detect_tree_finish:
            count += 1
            if color_identify(img, 'green')[0] != -1:
                have_detect_tree = True
                detect_tree_finish = True                
                count = 0
            if count > 5:
                have_detect_tree = False
                detect_tree_finish = True                
                count = 0
                
    cv2.line(img, (0, 200), (500, 200), (255, 0, 255), 2)
    cv2.line(img, (0, 280), (500, 280), (255, 0, 255), 2)
    
    return img

if __name__ == '__main__':
    Board.setBusServoPulse(19, servo1, 500)
    Board.setBusServoPulse(20, servo2, 500)
    Board.setBusServoPulse(19, servo1, 500)
    Board.setBusServoPulse(20, servo2, 500)
    AGC.runAction(stand)

    my_camera = Camera.Camera()
    my_camera.camera_open()
    while True:
        img = my_camera.frame
        if img is not None:
            frame = img.copy()
            Frame = run(frame)           
            if debug_mode != 0:
                cv2.imshow('Frame', Frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
        else:
            time.sleep(0.01)
    my_camera.camera_close()
    cv2.destroyAllWindows()
