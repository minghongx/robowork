import types
# import functools
from abc import ABC, abstractmethod
from concurrent.futures import ThreadPoolExecutor
from collections import deque
from multiprocessing import Event
from time import sleep
# from operator import methodcaller

# import psutil
import numpy as np
import cv2 as cv
from transitions import Machine

from hiwonder_puppypipro_servo import PWMServos
from locomotion_control import ActionGroups
from line_following_methods import row_of_pixels_method

from developing.object_detection_methods import _calc_max_contour_area

# TODO: Obsolete these modules
from deprecated.HiwonderPuppy import PUPPY, BusServoParams
from deprecated.ServoCmd import runActionGroup
from deprecated.BusServoControl import setBusServoPulse


class PreliminaryCompetitionRequirements(ABC):

    @abstractmethod
    def follow_the_30mm_black_line(self):
        pass

    @abstractmethod
    def detect_the_blue_pet_door(self):
        pass

    @abstractmethod
    def pass_thru_the_blue_pet_door(self):
        pass

    @abstractmethod
    def detect_the_yellow_demarcation_line(self):
        pass

    @abstractmethod
    def climb_the_curb(self):
        pass

    @abstractmethod
    def descend_the_curb(self):
        pass

    @abstractmethod
    def detect_the_black_cross(self):
        pass


class PreliminaryCompetitionStrategy(PreliminaryCompetitionRequirements, Machine):

    # def __enter__(self):
    #     self.pool = ThreadPoolExecutor(max_workers=8)
    #     self.cap = cv.VideoCapture(0)
    #     return self
    #
    # def __exit__(self, exc_type, exc_val, exc_tb):  # FIXME: with 整块的代码逻辑不对
    #     self.pool.shutdown()
    #     self.cap.release(); print('camera released')
    #     cv.destroyAllWindows()
    #     ActionGroups().unload()

    def __init__(self):

        self.pool = ThreadPoolExecutor(max_workers=10)

        # Finite State Machine
        self.states = [
            {'name': 'idle'},
            {'name': 'detect the blue pet door'},
            {'name': 'pass thru the blue pet door'},
            {'name': 'detect the yellow demarcation line'},
            {'name': 'on the plate detect the yellow demarcation line'},
            {'name': 'detect the black cross'},
        ]
        self.transitions = [
            {'trigger':'failed', 'source':'*', 'dest':'idle', 'after':'lay_idle'},
            {'trigger':'start', 'source':'idle', 'dest':'detect the blue pet door', 'before':['buffer_frames', 'prepare', 'print_state'], 'after':['follow_the_30mm_black_line', 'detect_the_blue_pet_door']},
            {'trigger':'close_to_the_door', 'source':'detect the blue pet door', 'dest':'pass thru the blue pet door', 'after':'pass_thru_the_blue_pet_door'},
            {'trigger':'thru_the_door', 'source':'pass thru the blue pet door', 'dest':'detect the yellow demarcation line', 'after':'detect_the_yellow_demarcation_line'},
            {'trigger':'close_to_the_curb', 'source':'detect the yellow demarcation line', 'dest':'on the plate detect the yellow demarcation line', 'before':'climb_the_curb', 'after':'detect_the_yellow_demarcation_line'},
            {'trigger':'close_to_the_curb', 'source':'on the plate detect the yellow demarcation line', 'dest':'detect the black cross', 'before':'descend_the_curb', 'after':'detect_the_black_cross'},
            {'trigger':'crossed_the_finish_line', 'source':'detect the black cross', 'dest':'idle', 'after':'lay_idle'},
        ]
        Machine.__init__(self, states=self.states, transitions=self.transitions, initial='idle')

        # TODO: encapsulate a Camera Class
        self.cap = cv.VideoCapture(0)
        self.camera = PWMServos(12)
        self.frames = deque(maxlen=3)

        self.start_following_line = Event()
        self.gait = PUPPY(setServoPulse=setBusServoPulse, servoParams=BusServoParams())  # FIXME: 自己设计 gait

    class submit_to_the_pool:
        def __init__(self, func):
            self.func = func

        def __call__(self, instance, *args, **kwargs):
            instance.pool.submit(self.func, instance, *args, **kwargs)

        def __get__(self, instance, instancetype):
            if instance is None:
                return self  # Accessed from class, return unchanged
            return types.MethodType(self, instance)  # Accessed from instance, bind to instance

    @submit_to_the_pool
    def lay_idle(self):  # FIXME
        self.start_following_line.clear()  # stop following line
        self.camera.set_pwm_servo_pulse(1500)
        ActionGroups.unload()  # unload servos to save power
        print("is lying idle")

    @submit_to_the_pool
    def prepare(self):
        self.gait.stance_config(self._stance(0, 0, -15, 2), pitch=0, roll=0)  # 标准站姿
        self.gait.gait_config(overlap_time=0.1, swing_time=0.15, z_clearance=3)
        self.gait.run()  # 启动

    @submit_to_the_pool
    def print_state(self):
        while True:
            sleep(2)
            print(self.state)

    @submit_to_the_pool
    def buffer_frames(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                continue
            self.frames.append(frame)

    @submit_to_the_pool
    def follow_the_30mm_black_line(self):  # FIXME: 自己设计 gait

        # FIXME
        camera_position = 2300
        self.camera.set_pwm_servo_pulse(camera_position)
        self.start_following_line.set()

        while True:
            self.start_following_line.wait()

            sleep(0.1)

            try:
                frame = self.frames.pop()
            except IndexError:
                continue

            px_deviation = row_of_pixels_method(frame, 420)
            if abs(px_deviation) < 68:
                self.gait.move(x=12, y=0, yaw_rate=0)  # go forward
                camera_position -= 60 if camera_position > 1600 else 0
                self.camera.set_pwm_servo_pulse(camera_position)  # FIXME
            if px_deviation >= 68:
                self.gait.move(x=7, y=0, yaw_rate=-25 / 57.3)  # turn right
                camera_position += 50 if camera_position < 2300 else 0
                self.camera.set_pwm_servo_pulse(camera_position)  # FIXME
            if px_deviation <= -68:
                self.gait.move(x=7, y=0, yaw_rate=25 / 57.3)  # turn left
                camera_position += 50 if camera_position < 2300 else 0
                self.camera.set_pwm_servo_pulse(camera_position)  # FIXME

    @submit_to_the_pool
    def detect_the_blue_pet_door(self):
        shape = cv.cvtColor(cv.imread("pet_door_standard.jpg"), cv.COLOR_BGR2GRAY)
        while True:

            sleep(0.1)

            try:
                frame = self.frames.pop()
            except IndexError:
                continue

            hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
            blurred_hsv = cv.GaussianBlur(hsv, (3, 3), 3)
            blue = cv.inRange(blurred_hsv, (100, 70, 46), (124, 255, 255))
            opened_blue = cv.morphologyEx(blue, cv.MORPH_OPEN, (6, 6))
            contours, _ = cv.findContours(opened_blue, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_TC89_L1)
            try:
                presumable_door_contour, area = _calc_max_contour_area(contours)
            except ValueError:
                continue
            x, y, w, h = cv.boundingRect(presumable_door_contour)
            disparity = cv.matchShapes(shape, opened_blue[y:y+h, x:x+w], cv.CONTOURS_MATCH_I1, 0)

            if disparity < 0.003 and area > 60000:
                break
        self.close_to_the_door()

    @submit_to_the_pool
    def pass_thru_the_blue_pet_door(self):  # FIXME: 自己设计 gait
        sleep(2)
        self.camera.set_pwm_servo_pulse(2100)
        self.gait.stance_config(self._stance(0, 0, -12, 2), pitch=0, roll=0)  # 趴下
        self.gait.gait_config(overlap_time=0.1, swing_time=0.15, z_clearance=2)
        sleep(9)
        self.gait.stance_config(self._stance(0, 0, -15, 2), pitch=0, roll=0)  # 过完门站起来.
        self.thru_the_door()

    @submit_to_the_pool
    def detect_the_yellow_demarcation_line(self):
        while True:

            sleep(0.1)

            try:
                frame = self.frames.pop()
            except IndexError:
                continue

            hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
            blurred_hsv = cv.GaussianBlur(hsv, (3, 3), 3)
            yellow = cv.inRange(blurred_hsv, (20, 44, 44), (38, 255, 255))
            opened_blue = cv.morphologyEx(yellow, cv.MORPH_OPEN, (6, 6))
            contours, _ = cv.findContours(opened_blue, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_TC89_L1)
            try:
                _, area = _calc_max_contour_area(contours)
            except ValueError:
                continue
            if area > 100000:
                break
        self.close_to_the_curb()

    @submit_to_the_pool
    def climb_the_curb(self):  # FIXME: 自己设计动作
        self.start_following_line.clear()

        self.gait.move(x=5, y=0, yaw_rate=0)
        self.gait.move_stop()
        sleep(0.2)
        runActionGroup('coord_up_stair_1')
        sleep(8)
        self.gait.stance_config(self._stance(0, 0, -13, -4), pitch=-20 / 57.3, roll=0)
        self.gait.gait_config(overlap_time=0.2, swing_time=0.4, z_clearance=2)
        self.gait.move_stop()
        sleep(0.1)
        self.gait.move(x=2.5, y=0, yaw_rate=0)
        sleep(4.5)
        self.gait.move_stop()
        sleep(0.2)
        runActionGroup('coord_up_stair_2')
        sleep(7)
        self.gait.stance_config(self._stance(0, 0, -13, 0), pitch=0, roll=0)
        self.gait.gait_config(overlap_time=0.1, swing_time=0.2, z_clearance=1.5)
        sleep(5.1)

        self.start_following_line.set()

    @submit_to_the_pool
    def descend_the_curb(self):  # FIXME: 自己设计动作
        self.start_following_line.clear()

        self.gait.move_stop()
        sleep(0.3)
        runActionGroup('coord_down_stair')
        sleep(6)
        self.gait.stance_config(self._stance(0, 0, -13, 4), pitch=20 / 57.3, roll=0)
        self.gait.gait_config(overlap_time=0.1, swing_time=0.2, z_clearance=2)
        self.gait.move_stop()
        sleep(0.2)
        self.gait.move(x=3.7, y=0, yaw_rate=0)
        sleep(4.5)
        self.gait.stance_config(self._stance(0, 0, -15, 2))
        self.gait.gait_config(overlap_time=0.1, swing_time=0.15, z_clearance=3)
        self.gait.move_stop()
        sleep(0.2)

        self.start_following_line.set()

    @submit_to_the_pool
    def detect_the_black_cross(self):
        shape = cv.cvtColor(cv.imread("cross_standard.png"), cv.COLOR_BGR2GRAY)
        while True:

            sleep(0.1)

            try:
                frame = self.frames.pop()
            except IndexError:
                continue

            _, otsu_binary_img = cv.threshold(cv.cvtColor(frame, cv.COLOR_BGR2GRAY), 0, 255, cv.THRESH_BINARY_INV + cv.THRESH_OTSU)
            opened_otsu_binary_img = cv.morphologyEx(otsu_binary_img, cv.MORPH_OPEN, (7, 7))
            contours, _ = cv.findContours(opened_otsu_binary_img, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_TC89_L1)
            try:
                presumable_door_contour, area = _calc_max_contour_area(contours)
            except ValueError:
                continue
            x, y, w, h = cv.boundingRect(presumable_door_contour)
            disparity = cv.matchShapes(shape, opened_otsu_binary_img[y:y+h, x:x+w], cv.CONTOURS_MATCH_I1, 0)
            if disparity < 0.003:
                break
        self.crossed_the_finish_line()

    @staticmethod  # FIXME: 自己设计 gait
    def _stance(x=0, y=0, z=-15, x_shift=2):  # 单位cm
        # x_shift 越小, 走路越前倾, 越大越后仰. 通过调节 x_shift 可以调节小狗走路的平衡
        return np.array([
                            [x + x_shift, x + x_shift, -x + x_shift, -x + x_shift],
                            [y, y, y, y],
                            [z, z, z, z],
                        ])  # 此array的组合方式不要去改变


'''
Ref
curb and kerb
add_done_callback(fn)：为该 Future 代表的线程任务注册一个“回调函数”，当该任务成功完成时，程序会自动触发该 fn 函数。
https://docs.python.org/zh-cn/3/library/multiprocessing.html#multiprocessing.pool.Pool.terminate
'''