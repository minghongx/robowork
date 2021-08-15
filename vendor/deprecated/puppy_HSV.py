from time import sleep
import sys
import numpy as np
from transitions import Machine
import cv2
import math
import threading

sys.path.append('/home/pi/puppy/')
from HiwonderPuppy import PUPPY, BusServoParams

sys.path.append('/home/pi/PuppyPi_PC_Software/')
from ServoCmd import *

sys.path.append('/home/pi/PuppyPi/')
from HiwonderSDK.Board import *
import HiwonderSDK.Board as Board
from HSVConfig import *
import Camera

sys.path.append('/home/pi/Desktop/AX-remoteProjs/developing/')
from line_following_methods import calc_deviation_from_a_row_of_pixels

import HiwonderSDK.Sonar as Sonar
#摄像头参数
SERVO_VALUE_LOOK_DOWN = 2450
SERVO_VALUE_LOOK_UP = 1500
SERVO_VALUE_DEVIATION =40
def calc_max_contour_area(contours):

    if not contours: raise ValueError

    contours_area = np.array([cv2.contourArea(contour) for contour in contours])
    index = np.argmax(contours_area)
    '''
    Contours is a Python list of all the contours in the image. 
    Each individual contour is a Numpy array of (x,y) coordinates of boundary points of the object.''''''
    '''
    return contours[index], contours_area[index]


def calc_longest_contour(contours):
    return max(contours, key=len)

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
puppy.run() # 启动
Board.setPWMServoPulse(1, SERVO_VALUE_LOOK_DOWN + SERVO_VALUE_DEVIATION, 100)
puppy.move_stop()

###########################################################################
my_camera= Camera.Camera()
my_camera.camera_open()

camera_pos=2500

def line_tracking(deviation,enable_PWM):
    global camera_pos
    #print(deviation)
    if deviation is None:
        time.sleep(0.01)
        return
    sleep(0.003)
    if abs(deviation) < 65:
        puppy.move(x=12, y=0, yaw_rate=0)  # go forward
        camera_pos -=27 if camera_pos > 1600 else 0
        if enable_PWM: Board.setPWMServoPulse(1, camera_pos, 100)
    if deviation >= 65:
        puppy.move(x=7, y=0, yaw_rate=-25 / 57.3)  # turn right
        camera_pos += 22 if camera_pos < 2300 else 0
        if enable_PWM: Board.setPWMServoPulse(1, camera_pos, 100)
    if deviation <= -65:
        puppy.move(x=7, y=0, yaw_rate=25 / 57.3)  # turn left
        camera_pos += 22 if camera_pos < 2300 else 0
        if enable_PWM: Board.setPWMServoPulse(1, camera_pos, 100)

class ImageProc:
    def __init__(self):
        self.block = False
        # 图像处理的各功能使能标记
        self.block_target_color = 'blue'
        self.enable_line = False  # 使能巡线处理
        self.enable_block = False  # 使能色块处理
        self.enable_PWM = True
        self.img_gate_standard=cv2.cvtColor(cv2.imread('gate_standard.jpg'), cv2.COLOR_BGR2GRAY)
        self.img_cross_standard=cv2.cvtColor(cv2.imread('cross_standard.png'), cv2.COLOR_BGR2GRAY)
        self.blue_area=0
        self.deviation=320
        # 启动图像处理线程
        threading.Thread(target=self.run, daemon=True).start()

    def tracking(self, frame):
        try:
            self.deviation = calc_deviation_from_a_row_of_pixels(frame, 420)
        except:
            pass
        line_tracking(self.deviation,self.enable_PWM)
#        return calc_deviation_from_a_row_of_pixels(frame, 350)  # FIXME: ordinate
    
    def block_detect(self, img, color):
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        img_gb = cv2.GaussianBlur(img_hsv, (3, 3), 3)
        mask = cv2.inRange(img_gb, color_range[color][0], color_range[color][1])
        if color == 'blue':
            eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (6, 6)))  # 腐蚀
            dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 膨胀
            contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[0]  # 找出轮廓
            if not contours:  # 没有找到轮廓
                return False
            cnt_large, area = calc_max_contour_area(contours)  # 找出最大轮廓
            x,y,w,h = cv2.boundingRect(cnt_large)
            dist = cv2.matchShapes(self.img_gate_standard, dilated[y:y+h,x:x+w], cv2.CONTOURS_MATCH_I1, 0)
            if (dist < 0.01 and w*h > 60000) or w*h > 190000 or area > 75000:
                self.blue_area=w*h if area < 75000 else 240000
                return True
            else:
                return False
        
        if color == 'yellow':
            eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (6, 6)))  # 腐蚀
            dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 膨胀
            contours, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)  # 找出轮廓
            if not contours:  # 没有找到轮廓
                return False
            cnt_large, area = calc_max_contour_area(contours)  # 找出最大轮廓
            if area > 40000:
                return True
            else:
                return False

        if color == 'black':
            _, otsu_binary_img = cv2.threshold(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
            opened_otsu_binary_img = cv2.morphologyEx(otsu_binary_img,cv2.MORPH_OPEN,(7,7))
            contours, _ = cv2.findContours(opened_otsu_binary_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)  # 找出轮廓
            if not contours:  # 没有找到轮廓
                return False
            cnt_large, area = calc_max_contour_area(contours)  # 找出最大轮廓
            x, y, w, h = cv2.boundingRect(cnt_large)
            dist = cv2.matchShapes(self.img_cross_standard, opened_otsu_binary_img[y:y + h, x:x + w], cv2.CONTOURS_MATCH_I1, 0)
            if dist < 0.009 and area > 110000:
                return True
            else:
                return False

    def run(self):
        while True:
            fra = my_camera.frame
            if fra is None:
                sleep(0.01)
                continue
            img = fra.copy()
            sleep(0.00001)
            if self.enable_line: self.tracking(img)
            self.block = self.block_detect(img, self.block_target_color) if self.enable_block else None
            #cv2.imshow('img', img)
            #cv2.waitKey(1)

###########################################################################################
class LogicProc:
    def __init__(self, image_proc: ImageProc):
        self.image_proc = image_proc
        self.event = None
        self.states = [  # 状态列表
            {'name': 'IDLE','on_enter': 'on_enter_stop'},  # 空闲状态, 启动后默认
            {'name': 'FIND_CROSS', 'on_enter': 'on_enter_find_cross'},
            {'name': 'FIND_BRIDGE', 'on_enter': 'on_enter_find_bridge'},
            {'name': 'CROSS_BRIDGE', 'on_enter': 'on_enter_cross_bridge'},
            {'name': 'FIND_DOOR', 'on_enter': 'on_enter_find_door'},
            {'name': 'THROUGH_DOOR', 'on_enter': 'on_enter_through_door', 'on_exit': 'on_exit_through_door'},
        ]
        self.operations = {  # 各个状态主操作
            "IDLE":self.idle,
            "FIND_CROSS": self.find_cross_action,
            "FIND_BRIDGE": self.find_bridge_action,
            "CROSS_BRIDGE": self.cross_bridge_action,
            "FIND_DOOR": self.find_door_action,
            "THROUGH_DOOR": self.through_door_action,
        }
        self.transitions = [  # 状态转换关系
            # 开始寻找门
            {'trigger': 'start', 'source': 'IDLE', 'dest': 'FIND_DOOR'},
            # 找到门, 过门
            {'trigger': 'found_door', 'source': 'FIND_DOOR', 'dest': 'THROUGH_DOOR'},
            # 完成过门， 找桥
            {'trigger': 'through_door', 'source': 'THROUGH_DOOR', 'dest': 'FIND_BRIDGE'},
            # 找到桥, 过桥
            {'trigger': 'found_bridge', 'source': 'FIND_BRIDGE', 'dest': 'CROSS_BRIDGE'},
            # 完成过桥, 找cross
            {'trigger': 'crossed_bridge', 'source': 'CROSS_BRIDGE', 'dest': 'FIND_CROSS'},
            # 找到cross, 结束
            {'trigger': 'finished', 'source': 'FIND_CROSS', 'dest': 'IDLE'}
        ]
        self.machine = Machine(model=self,
                               states=self.states,
                               transitions=self.transitions,
                               initial='IDLE',
#                               initial="CROSS_BRIDGE",
                               send_event=True,
                               before_state_change=self.state_change,
                               queued=True)

    def state_change(self, event):
        self.event = event

    def poll(self):
        if self.state in self.operations:
            self.operations[self.state]()

    def switch_line_tracking(self):
        if self.image_proc.enable_line == True:
            self.image_proc.enable_line = False
            puppy.move_stop()
            sleep(0.01)
        else:
            puppy.move_stop()
            sleep(0.01)
            self.image_proc.enable_line = True


    def down_stair(self):
        
        puppy.move_stop()
        sleep(0.3)

        runActionGroup('coord_down_stair')
        sleep(6)

        puppy.stance_config(stance=stance(0, 0, -13, 4), pitch=20 / 57.3, roll=0)
        puppy.gait_config(overlap_time=0.1, swing_time=0.2, z_clearance=2)
        puppy.move_stop()
        sleep(0.2)
        
        puppy.move(x=3.7, y=0, yaw_rate=0)
        sleep(4.5)
        '''
        puppy.stance_config(stance=stance(0, 0, -15, 2), pitch=0, roll=0)
        puppy.gait_config(overlap_time=0.2, swing_time=0.3, z_clearance=2)
        puppy.move_stop()
        sleep(0.2)
        '''
        puppy.stance_config(stance=stance(0, 0, -15, 2))
        puppy.gait_config(overlap_time=0.1, swing_time=0.15, z_clearance=3)
        puppy.move_stop()
        sleep(0.2)
        
        
        

    def idle(self):
        puppy.move(x=0, y=0, yaw_rate = 0)
        puppy.move_stop()

    def on_enter_find_door(self,event):
        puppy.stance_config(stance(0,0,-15,2), pitch = 0, roll = 0)
        puppy.move_stop()
        self.image_proc.block_target_color = 'blue'  # 设置色块识别目标颜色
        self.image_proc.block = None
        self.image_proc.enable_block = True  # 启动色块识别
        print("On enter find door")
        self.switch_line_tracking()
        sleep(5)

        
    def find_door_action(self):
        sleep(0.01)
        if self.image_proc.block:
            sleep((-0.000204 * self.image_proc.blue_area + 70) / 10-0.5 )
            return self.found_door()
        return

    def on_enter_through_door(self,event):
        self.image_proc.enable_PWM=False
        Board.setPWMServoPulse(1, 2100, 100)
        sleep(0.1)
        self.image_proc.enable_block = False
        self.switch_line_tracking()
        puppy.stance_config(stance(0, 0, -12, 2), pitch=0, roll=0)  # 趴下
        puppy.gait_config(overlap_time=0.1, swing_time=0.15, z_clearance=2)
        self.switch_line_tracking()
        sleep(1.8)
        print("On enter through door")

    def through_door_action(self):
        Board.setPWMServoPulse(1, 2100, 100)
        sleep(8)
        return self.through_door()

        
    def on_exit_through_door(self,event):
        self.switch_line_tracking()
        puppy.stance_config(stance(0,0,-15,2), pitch = 0, roll = 0)# 过完门站起来.
        self.switch_line_tracking()
        sleep(5)
        print("On exit through door")
        
        
    def on_enter_find_bridge(self,event):
        self.image_proc.block_target_color = 'yellow'
        self.image_proc.block = None
        self.image_proc.enable_block = True
        print("On enter find bridge")
        sleep(1)

    def find_bridge_action(self):
        sleep(0.005)
        if self.image_proc.block:
            sleep(0.9)
            return self.found_bridge()
        return

    def on_enter_cross_bridge(self,event):
        self.switch_line_tracking()
        print("On enter cross bridge")
        puppy.move(x=5, y=0, yaw_rate = 0)
        puppy.move_stop()
        sleep(0.2)
        runActionGroup('coord_up_stair_1')
        sleep(8)
        puppy.stance_config(stance=stance(0,0,-13,-4), pitch = -20/57.3, roll = 0)
        puppy.gait_config(overlap_time = 0.2, swing_time = 0.4, z_clearance = 2)
        puppy.move_stop()
        sleep(0.1)
        puppy.move(x=2.5, y=0, yaw_rate = 0)
        sleep(4.5)
        puppy.move_stop()
        sleep(0.2)
        runActionGroup('coord_up_stair_2')
        sleep(7)      
        puppy.stance_config(stance=stance(0,0,-13,0), pitch = 0, roll = 0)
        puppy.gait_config(overlap_time = 0.1, swing_time = 0.2, z_clearance = 1.5)
        self.switch_line_tracking()
        sleep(5.1)

    def cross_bridge_action(self):
        sleep(0.005)
        if self.image_proc.block:
            sleep(2.12)
            self.switch_line_tracking()
            self.down_stair()
            self.switch_line_tracking()
            return self.crossed_bridge()
        return

    def on_enter_find_cross(self,event):
        print("On enter find cross")
        puppy.stance_config(stance=stance(0,0,-15,2), pitch = 0, roll = 0)
        puppy.gait_config(overlap_time = 0.1, swing_time = 0.15, z_clearance = 3)
        sleep(5)
        self.image_proc.block_target_color = 'black'
        self.image_proc.block = None
        self.image_proc.enable_block = True

    def find_cross_action(self):
        sleep(0.005)
        if self.image_proc.block:
            print("find cross")
            sleep(5)
            return self.finished()
        return

    def on_enter_stop(self,event):
        my_camera.camera_close()
        cv2.destroyAllWindows()
        self.switch_line_tracking()
        
if __name__ == '__main__':
    img_proc = ImageProc()
    logic_proc = LogicProc(img_proc)
    logic_proc.start()
    #logic_proc.crossed_bridge()
    while True:
        logic_proc.poll()
