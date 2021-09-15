#!/usr/bin/env python3
import logging
import math
import threading
import time

import cv2
import hiwonder.ActionGroupControl as AGC
import numpy as np
from hiwonder import Board
from transitions import Machine
# from transitions.extensions import GraphMachine as Machine
import LABConfig

DEBUG = 1
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger('match')
logger.setLevel(logging.DEBUG)

# 头部舵机的参数
PITCH_SERVO_ID = 20
PITCH_SERVO_POS_INIT = LABConfig.servo2
PITCH_SERVO_POS_LINE_TRACKING = 200
PITCH_SERVO_POS_BLOCK_FINDING = 338
PITCH_SERVO_POS_DOOR_FINDING = 200
PITCH_SERVO_POS_SHOT_BALL = 300
PITCH_SERVO_POS_FIND_DOOR = 390
PITCH_SERVO_POS_THROUGH_DOOR = 260

YAW_SERVO_ID = 19
YAW_SERVO_POS_INIT = LABConfig.servo1
YAW_SERVO_POS_LINE_TRACKING = 500
YAW_SERVO_POS_BLOCK_FINDING = 300
YAW_SERVO_POS_DOOR_FINDING = 420
YAW_SERVO_POS_FIND_DOOR = 270
YAW_SERVO_POS_THROUGH_DOOR = 500
YAW_SERVO_POS_SHOT_BALL = 500

# 相机的设置参数
IMAGE_SIZE = 320, 320
IMAGE_CENTER_X = 160
CAP_SIZE = 480, 320
CAP_PROPS = (
    (cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')),
    (cv2.CAP_PROP_SATURATION, 0.5),  # saturation
    (cv2.CAP_PROP_CONTRAST, 1.2),
    (cv2.CAP_PROP_FRAME_WIDTH, CAP_SIZE[0]),
    (cv2.CAP_PROP_FRAME_HEIGHT, CAP_SIZE[1]),
    (cv2.CAP_PROP_FPS, 30)  # framerate
)

# 巡线的相关参数
LINE_TRACKING_COLOR = 'black'  # 线的颜色
LINE_TRACKING_AREA_THRESHOLD = 5
LINE_TRACKING_ROI_LST = [  # [ROI, weight]
    (0, 30, 0, IMAGE_SIZE[1], 0.1),
    (60, 90, 0, IMAGE_SIZE[1], 0.2),
    (120, 150, 0, IMAGE_SIZE[1], 0.7)
]
LINE_TRACKING_TURN_THRESHOLD = 20  # 巡线中进行转向的阈值

# 颜色方块的相关参数
BLOCK_AREA_THRESHOLD = 500  # 色块最小面积阈值
BLOCK_START_CLOSING_X = 45  # 判断开始执行接近方块逻辑的坐标阈值
TACK_UP_BLOCK_Y = 215  # 判断抱起方块的Y坐标 阈值


def get_matrix(p="cal3.npz"):
    """
    从文件中加载相机标定数据
    :param p: 文件路径
    :return:
    """
    np_load = np.load(p, allow_pickle=True)['arr_0']
    mtx = np_load[0]
    dist = np_load[1]
    return mtx, dist


def roi_to_box(roi):
    return np.array([(roi[0], roi[2]), (roi[1], roi[2]), (roi[1], roi[3]), (roi[0], roi[3])])


def get_area_max_contour(contours, threshold):
    """
    获取面积最大的轮廓
    :param contours: 要比较的轮廓的列表
    :param threshold: 轮廓要求的最小面积阈值
    :return:面积最大的轮廓, 面积最大的轮廓的面积值
    """
    contours = map(lambda x: (x, math.fabs(cv2.contourArea(x))), contours)  # 计算所有轮廓面积
    contours = list(filter(lambda x: x[1] > threshold, contours))
    if len(contours) > 0:
        return max(contours, key=lambda x: x[1])  # 返回最大的轮廓
    else:
        return None, None


class ImageProc:
    def __init__(self, cap_index=-1):
        # 图像处理的结果
        self.block = None
        self.line = None
        self.shot_ball = None

        # 图像处理的各功能使能标记
        self.enable_shot_ball = False
        self.enable_line = False  # 使能巡线处理
        self.enable_block = False  # 使能色块处理

        self.block_target_color = 'red'

        # 读取各种颜色的阈值
        self.color_ranges = LABConfig.color_range

        # 读取相机标定数据
        mtx, dist = get_matrix()
        new_mtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (480, 320), 1, (480, 320))
        map_x, map_y = cv2.initUndistortRectifyMap(mtx, dist, None, new_mtx, (480, 320), 5)
        self.cal_roi_x, self.cal_roi_y, self.cal_roi_w, self.cal_roi_h = roi
        self.mtx, self.dist, self.new_mtx = mtx, dist, new_mtx

        # 打开相机并设置参数
        self.cap = cv2.VideoCapture(cap_index)
        for prop_name, prop_data in CAP_PROPS:
            self.cap.set(prop_name, prop_data)

        # 启动图像处理线程
        threading.Thread(target=self.run, daemon=True).start()

    def tracking(self, img, img_lab):
        """
        :param img: 要处理的原始图像 bgr 空间
        :param img_lab: 要处理的原始图像 lab 空间
        :return:
        """
        n = 0
        centers = []
        mask = cv2.inRange(img_lab,
                           self.color_ranges[LINE_TRACKING_COLOR][0],
                           self.color_ranges[LINE_TRACKING_COLOR][1])  # 对原图像和掩模进行位运算
        opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))  # 开运算
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))  # 闭运算
        if DEBUG > 0:
            cv2.imshow('closed' + str(n), closed)
        max_width = 0
        # 将图像分割成上中下三个部分
        for i, (start_y, end_y, start_x, end_x, weight) in enumerate(LINE_TRACKING_ROI_LST):
            n += 1
            img_blobs = closed[start_y:end_y, start_x:end_x]
            cnts = cv2.findContours(img_blobs, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[-2]  # 找出所有轮廓
            cnt_large, area = get_area_max_contour(cnts, LINE_TRACKING_AREA_THRESHOLD)  # 找到最大面积的轮廓
            if cnt_large is not None:  # 如果轮廓不为空
                rect = cv2.minAreaRect(cnt_large)  # 最小外接矩形
                box = np.int0(cv2.boxPoints(rect))  # 最小外接矩形的四个顶点
                box[:, 1] = box[:, 1] + start_y  # 据小图内偏移计算大图的上的坐标
                cv2.drawContours(img, [box], -1, (0, 0, 255), 2)  # 画出四个点组成的矩形
                # 获取矩形的对角点
                pt1_x, pt1_y = box[0, 0], box[0, 1]
                pt3_x, pt3_y = box[2, 0], box[2, 1]
                center_x, center_y = (pt1_x + pt3_x) / 2, (pt1_y + pt3_y) / 2  # 中心点
                cv2.circle(img, (int(center_x), int(center_y)), 5, (0, 0, 255), -1)  # 画出中心点
                centers.append((center_x, center_y, weight))
                width = max(rect[1])
                if rect[1][0] > max_width and i == 2:
                    max_width = width

        # 按权重不同对上中下三个中心点进行求和
        center_x_sum = sum(c[0] * c[2] for c in centers)
        weight_sum = sum(c[2] for c in centers)

        if not weight_sum == 0:
            # 求最终得到的中心点
            line_center_x = int(center_x_sum / weight_sum)
            cv2.circle(img, (line_center_x, int(LINE_TRACKING_ROI_LST[1][1])), 10, (0, 255, 255), -1)  # 画出中心点
            return line_center_x, max_width
        else:
            return None

    def shot_ball_proc(self, img, img_lab):
        """
        射门的视觉处理,
        找到龙门的红色区域, 取最小外接矩形,以该矩形的下边作为航向的参照, 使机器人实现始终垂直于改边
        找到小球, 取最小外接园, 以该园的圆心作为机器人左右偏移及前后移动的的参照, 使小球置于机器人脚前
        :param img:
        :param img_lab:
        :return:
        """
        ball_color = 'purple'
        goal_color = 'goal'
        goal = self.block_detect(img, img_lab, goal_color)
        mask = cv2.inRange(img_lab, self.color_ranges[ball_color][0], self.color_ranges[ball_color][1])  # 对原图像和掩模进行位运算
        eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 腐蚀
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 膨胀
        contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓
        cnt_max, area = get_area_max_contour(contours, BLOCK_AREA_THRESHOLD)  # 找出最大轮廓

        if cnt_max is not None:
            ball = cv2.minEnclosingCircle(cnt_max)  # 获取最小外接圆的圆心以及半径
            cv2.circle(img, (int(ball[0][0]), int(ball[0][1])), int(ball[1]), (0, 255, 0), 2)
            cv2.circle(img, (int(ball[0][0]), int(ball[0][1])), 5, (0, 255, 0), -1)
            return ball, goal
        else:
            return None, None

    def block_detect(self, img, img_lab, color):
        """
        从画面中所有颜色方块
        :param img:  #bgr 空间的图片, 用于显示
        :param img_lab: #lab 空间的图像, 用于处理
        :return:
        """
        mask = cv2.inRange(img_lab, self.color_ranges[color][0], self.color_ranges[color][1])  # 对原图像和掩模进行位运算
        eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 腐蚀
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 膨胀
        contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓
        cnt_max, area = get_area_max_contour(contours, BLOCK_AREA_THRESHOLD)  # 找出最大轮廓

        if cnt_max is not None:
            rect = cv2.minAreaRect(cnt_max)  # 计算轮廓的最小外接矩形
            angle = rect[2]
            box = np.int0(cv2.boxPoints(rect))  # 最小外接矩形的四个顶点
            cv2.drawContours(img, [box], -1, (255, 200, 200), 2)  # 画出四个点组成的矩形
            # 获取矩形的对角点
            center_x = int(rect[0][0])
            center_y = min(box.tolist(), key=lambda p: p[1])[1]
            cv2.circle(img, (center_x, center_y), 5, (0, 255, 255), -1)  # 画出中心点
            return center_x, center_y, angle, area, color
        else:
            return None

    def run(self):
        while True:
            img = self.read_camera()
            img_gb = cv2.GaussianBlur(img, (3, 3), 5)
            img_lab = cv2.cvtColor(img_gb, cv2.COLOR_BGR2LAB)
            self.line = self.tracking(img, img_lab) if self.enable_line else None
            self.block = self.block_detect(img, img_lab, self.block_target_color) if self.enable_block else None
            self.shot_ball = self.shot_ball_proc(img, img_lab) if self.enable_shot_ball else None
            if DEBUG > 0:
                cv2.imshow('img', img)
            cv2.waitKey(10)

    def read_camera(self):
        """
        从相机中读取画面并将画面做畸变矫正
        :return: 矫正后的画面
        """
        _, img = self.cap.read()  # 这里不对异常做处理, 读不到摄像头程序会直接因异常退出
        remapped = cv2.undistort(img, self.mtx, self.dist, None, self.new_mtx)
        remapped = remapped[
                   # self.cal_roi_y:self.cal_roi_y + self.cal_roi_h - 50,
                   # self.cal_roi_x + 75:self.cal_roi_x + self.cal_roi_w - 140]
                   self.cal_roi_y: self.cal_roi_y + self.cal_roi_h - 50,
                   self.cal_roi_x + 30: self.cal_roi_x + self.cal_roi_w - 160]
        remapped = cv2.resize(remapped, IMAGE_SIZE)
        return remapped


class LogicProc:
    def __init__(self, image_proc: ImageProc):
        self.image_proc = image_proc
        self.last_state = None
        self.source_state = None
        self.delay = None
        self.event = None

        self.states = [  # 状态列表
            {'name': 'IDLE'},  # 空闲状态, 启动后默认
            {'name': 'FIND_MARK', 'on_enter': 'on_enter_find_mark', 'on_exit': 'on_exit_find_mark'},
            {'name': 'FIND_BLOCK', 'on_enter': 'on_enter_find_block', 'on_exit': 'on_exit_find_block'},
            {'name': 'FIND_BRIDGE', 'on_enter': 'on_enter_find_bridge'},
            {'name': 'CROSS_BRIDGE', 'on_enter': 'on_enter_cross_bridge'},
            {'name': 'FIND_DOOR', 'on_enter': 'on_enter_find_door'},
            {'name': 'THROUGH_DOOR', 'on_enter': 'on_enter_through_door', 'on_exit': 'on_exit_through_door'},
            {'name': 'SHOT_BALL', 'on_enter': 'on_enter_shot_ball'},
        ]
        self.operations = {  # 各个状态主操作
            "FIND_MARK": self.line_tracking,
            "FIND_BLOCK": self.find_block,
            "FIND_BRIDGE": self.line_tracking,
            "CROSS_BRIDGE": self.line_tracking,
            "FIND_DOOR": self.line_tracking,
            "THROUGH_DOOR": self.through_door_tracking,
            "SHOT_BALL": self.shot_ball
        }

        self.transitions = [  # 状态转换关系
            # 开始寻找彩柱的标记(黑色横线)
            {'trigger': 'start', 'source': 'IDLE', 'dest': 'FIND_MARK'},
            # 找到了彩柱标记, 开始寻找彩柱
            {'trigger': 'found_mark_block', 'source': 'FIND_MARK', 'dest': 'FIND_BLOCK'},
            # 完成彩柱任务, 找桥梁
            {'trigger': 'found_block', 'source': 'FIND_BLOCK', 'dest': 'FIND_BRIDGE'},
            # 找到桥, 过桥
            {'trigger': 'found_bridge', 'source': 'FIND_BRIDGE', 'dest': 'CROSS_BRIDGE'},
            # 完成过桥, 找矮门
            {'trigger': 'crossed_bridge', 'source': 'CROSS_BRIDGE', 'dest': 'FIND_DOOR'},
            # 找到矮门, 巡线过门
            {'trigger': 'found_door', 'source': 'FIND_DOOR', 'dest': 'THROUGH_DOOR'},
            # 完成过矮门, 巡线找球
            {'trigger': 'through_door', 'source': 'THROUGH_DOOR', 'dest': 'FIND_MARK'},
            # 找到球, 调整,踢球
            {'trigger': 'found_mark_ball', 'source': 'FIND_MARK', 'dest': 'SHOT_BALL'},
            # 完成
            {'trigger': 'finished', 'source': 'SHOT_BALL', 'dest': 'IDLE'}
        ]

        self.machine = Machine(model=self,
                               states=self.states,
                               transitions=self.transitions,
                               initial='IDLE',
                               send_event=True,
                               before_state_change=self.state_change,
                               # show_auto_transitions=False,
                               # show_state_attributes=True,
                               queued=True)

    def state_change(self, event):
        self.event = event

    def poll(self):
        if self.state in self.operations:
            self.operations[self.state]()

    def on_enter_cross_bridge(self, event):
        """
        进入过桥过程
        :param event:
        :return:
        """
        Board.setBusServoPulse(PITCH_SERVO_ID, PITCH_SERVO_POS_LINE_TRACKING, 400)
        Board.setBusServoPulse(YAW_SERVO_ID, YAW_SERVO_POS_LINE_TRACKING, 400)
        AGC.runAction('stand')
        AGC.runAction('ladder_up')  # 运行上桥动作
        AGC.runAction('stand')
        self.delay = time.time() + 3  # 上桥后不立即进行下桥标志的识别防止误识别, 延时几秒, 提高可靠性

    def on_enter_find_door(self, event):
        """
        进入矮门查找设置相关参数及启用功能
        查找功能废弃, 直接定时走固定时间, 效果尚可
        :param event:
        :return:
        """
        self.image_proc.block_target_color = 'green'  # 查找绿色色块
        self.image_proc.block = None
        self.image_proc.line = None
        self.image_proc.enable_block = True  # 启动色块识别
        self.image_proc.enable_line = True
        Board.setBusServoPulse(PITCH_SERVO_ID, PITCH_SERVO_POS_LINE_TRACKING, 400)
        Board.setBusServoPulse(YAW_SERVO_ID, YAW_SERVO_POS_LINE_TRACKING, 400)
        self.delay = time.time() + 17  # 设置延时, 在超过这个时间点之前只做巡线, 超过之后, 侧头检测绿色矮门

    def on_enter_through_door(self, event):
        """
        进入过矮门,蹲低身子,摄像头角度做对应调整以能够正常巡线
        :param event:
        :return:
        """
        Board.setBusServoPulse(PITCH_SERVO_ID, PITCH_SERVO_POS_THROUGH_DOOR, 400)
        Board.setBusServoPulse(YAW_SERVO_ID, YAW_SERVO_POS_THROUGH_DOOR, 400)
        AGC.runAction('stand_low', send_again=True)
        self.delay = time.time() + 10  # 设置延时, 过矮使用专门动作, 超过此延时后认为已经过矮门, 进入下一步

    def on_exit_through_door(self, event):
        """
        退出过矮门
        :param event:
        :return:
        """
        self.image_proc.block = None
        self.image_proc.line = None
        self.image_proc.enable_block = False  # 关闭色块识别
        self.image_proc.enable_line = True
        AGC.runAction('stand')

    def on_enter_shot_ball(self, event):
        self.image_proc.enable_line = False
        self.image_proc.enable_shot_ball = True  # 启动色块识别
        Board.setBusServoPulse(PITCH_SERVO_ID, PITCH_SERVO_POS_SHOT_BALL, 400)  # 设置舵机角度
        Board.setBusServoPulse(YAW_SERVO_ID, YAW_SERVO_POS_SHOT_BALL, 400)
        time.sleep(0.5)

    def shot_ball(self):
        goal, ball = None, None
        ball = self.image_proc.shot_ball

        if ball is None:
            time.sleep(0.01)
            return
        ball, goal = ball

        if ball is not None:
            (ball_x, ball_y), ball_r = ball
            x_dist = ball_x - 205
            if ball_y < 220:
                AGC.runAction('go_forward_fast')
                return

            if x_dist > 20:
                AGC.runAction('right_move')
                return
            elif x_dist < -20:
                AGC.runAction('left_move')
                return
            else:
                pass

            if ball_y < 260:
                AGC.runAction('go_forward_one_step')
                return

        if goal is not None:
            x, y, angle, area, name = goal
            if -45 < angle < -10:
                AGC.runAction('turn_left_0413')
                return
            elif -90 < angle < -45:
                AGC.runAction('turn_right_0413')
                return
            else:
                pass

        AGC.runAction('right_shot')
        self.finished()

    def on_enter_find_block(self, event):
        """
        进入彩柱寻找, 这里几乎可以肯定能找到
        :param event:
        :return:
        """
        self.image_proc.block_target_color = 'red'  # 设置色块识别目标颜色
        self.image_proc.enable_block = True  # 启动色块识别
        Board.setBusServoPulse(PITCH_SERVO_ID, PITCH_SERVO_POS_BLOCK_FINDING, 400)  # 设置舵机角度
        Board.setBusServoPulse(YAW_SERVO_ID, YAW_SERVO_POS_BLOCK_FINDING, 400)
        time.sleep(0.5)

    def on_exit_find_block(self, event):
        """
        退出彩柱寻找
        :param event:
        :return:
        """
        self.image_proc.enable_block = False  # 启动色块识别

    def on_enter_find_bridge(self, event):
        """
        进入桥梁检测
        :param event:
        :return:
        """
        self.image_proc.block_target_color = 'yellow'
        self.image_proc.block = None
        self.image_proc.enable_block = True
        self.image_proc.enable_line = True
        Board.setBusServoPulse(PITCH_SERVO_ID, PITCH_SERVO_POS_LINE_TRACKING, 400)
        Board.setBusServoPulse(YAW_SERVO_ID, YAW_SERVO_POS_LINE_TRACKING, 400)
        time.sleep(0.5)

    def find_block(self):
        block = self.image_proc.block
        if block is None:
            time.sleep(0.01)
            return
        center_x, center_y, angle, area, color = block
        if area > 1000:
            Board.setBuzzer(1)
            time.sleep(2.1)
            Board.setBuzzer(0)
            self.time_st = time.time() + 10
            return self.found_block()
        time.sleep(0.01)
        return

    def on_exit_find_mark(self, event):
        """
        退出彩柱标记查找, 关闭巡线的图像处理
        :param event:
        :return:
        """
        self.image_proc.enable_line = False

    def on_enter_find_mark(self, event):
        """
        进入查找彩柱标记(黑色横线),巡线, 设置巡线时的舵机角度, 启动巡线的图像处理
        :param event:
        :return:
        """
        self.image_proc.line = None
        self.image_proc.enable_line = True
        Board.setBusServoPulse(PITCH_SERVO_ID, PITCH_SERVO_POS_LINE_TRACKING, 200)
        Board.setBusServoPulse(YAW_SERVO_ID, YAW_SERVO_POS_LINE_TRACKING, 200)
        time.sleep(0.25)

    def through_door_tracking(self):
        line = self.image_proc.line
        if line is None:
            time.sleep(0.01)
            return
        line_center_x, max_width = line
        if self.delay < time.time():
            return self.through_door()  # 完成过矮门
        else:
            dist = line_center_x - IMAGE_CENTER_X
            if -LINE_TRACKING_TURN_THRESHOLD <= dist <= LINE_TRACKING_TURN_THRESHOLD:
                AGC.runAction('go_forward_low', send_again=False)
            elif dist > LINE_TRACKING_TURN_THRESHOLD:
                AGC.runAction('go_turn_right_low', send_again=False)
            elif dist < -LINE_TRACKING_TURN_THRESHOLD:
                AGC.runAction('go_turn_left_low', send_again=False)
            else:
                pass
            time.sleep(0.01)

    def line_tracking(self):
        """
        巡线的逻辑
        :return:
        """
        line = self.image_proc.line
        if line is None:
            time.sleep(0.01)
            return
        line_center_x, max_width = line

        if self.delay < time.time():
            if max_width > 100:
                if self.is_FIND_MARK():
                    if self.event.transition.source == 'IDLE':
                        return self.found_mark_block()
                    elif self.event.transition.source == 'THROUGH_DOOR':
                        return self.found_mark_ball()
            if self.is_FIND_BRIDGE():  # 源状态为检测色块时检测桥梁
                block = self.image_proc.block
                if block is not None:
                    x, y, angle, area, name = block
                    if y > 100:
                        return self.found_bridge()
            if self.is_CROSS_BRIDGE():
                block = self.image_proc.block
                if block is not None:
                    x, y, angle, area, name = block
                    if y > 100:
                        AGC.runAction('go_forward_one_step')
                        AGC.runAction('ladder_down')
                        AGC.runAction('stand')
                        return self.crossed_bridge()
            if self.is_FIND_DOOR():
                Board.setBusServoPulse(PITCH_SERVO_ID, PITCH_SERVO_POS_FIND_DOOR, 200)
                Board.setBusServoPulse(YAW_SERVO_ID, YAW_SERVO_POS_FIND_DOOR, 200)
                time.sleep(0.5)
                block = self.image_proc.block
                logger.debug("find_door {}".format(block))
                Board.setBusServoPulse(PITCH_SERVO_ID, PITCH_SERVO_POS_LINE_TRACKING, 200)
                Board.setBusServoPulse(YAW_SERVO_ID, YAW_SERVO_POS_LINE_TRACKING, 200)
                time.sleep(0.4)
                self.image_proc.line = None
                self.delay = time.time() + 1.5
                if block is not None:
                    x, y, angle, area, name = block
                    if area > 23000:
                        return self.found_door()
                return

        dist = line_center_x - IMAGE_CENTER_X
        if -LINE_TRACKING_TURN_THRESHOLD <= dist <= LINE_TRACKING_TURN_THRESHOLD:
            AGC.runAction('go_forward_fast', send_again=False)
            logger.debug('go_forward_fast')
        elif LINE_TRACKING_TURN_THRESHOLD < dist < LINE_TRACKING_TURN_THRESHOLD + 50:
            AGC.runAction('go_turn_right_fast', send_again=False)
            logger.debug('go_turn_right_fast')
        elif dist >= LINE_TRACKING_TURN_THRESHOLD + 50:
            AGC.runAction('turn_right_0413', send_again=False)
        elif -LINE_TRACKING_TURN_THRESHOLD > dist > -LINE_TRACKING_TURN_THRESHOLD - 50:
            AGC.runAction('go_turn_left_fast', send_again=False)
            logger.debug('go_turn_left_fast')
        elif dist <= -LINE_TRACKING_TURN_THRESHOLD - 50:
            AGC.runAction('turn_left_0413', send_again=False)
        else:
            pass

        return


if __name__ == '__main__':
    Board.setBusServoPulse(20, PITCH_SERVO_POS_INIT, 500)
    Board.setBusServoPulse(19, YAW_SERVO_POS_INIT, 500)
    Board.setBuzzer(0)

    img_proc = ImageProc(-1)
    logic_proc = LogicProc(img_proc)
    AGC.runAction('1')
    logic_proc.delay = time.time() + 5
    # logic_proc.get_graph().draw('./工程赛程序状态图.png', prog='dot')
    logic_proc.start()
    # logic_proc.to_FIND_DOOR()
    # logic_proc.crossed_bridge()
    while True:
        logic_proc.poll()
