import types
# import functools
# from operator import methodcaller
from abc import ABC, abstractmethod
from concurrent.futures import ThreadPoolExecutor
from collections import deque
from multiprocessing import Event
from time import sleep

# import psutil
import numpy as np
import cv2 as cv
from transitions import Machine

# from vendor.mpu6050.mpu6050 import mpu6050
from vendor.deprecated.HiwonderPuppy import PUPPY, BusServoParams  # TODO: Obsolete these modules
from vendor.deprecated.ServoCmd import runActionGroup
from vendor.deprecated.BusServoControl import setBusServoPulse

from puppypi_pro.hiwonder_puppypipro_servo import PWMServos
from puppypi_pro.locomotion_control import ActionGroups
from puppypi_pro.line_following_methods import row_of_pixels_method
from puppypi_pro.object_detection_methods import _calc_max_contour_area


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

    def __enter__(self):

        # TODO: encapsulate a Camera Class
        self.__cap = cv.VideoCapture(-1)
        self.__camera = PWMServos(12)
        self.frames = deque(maxlen=2)

        self._pool = ThreadPoolExecutor(max_workers=8)

        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self._pool.shutdown()
        self.__cap.release()
        cv.destroyAllWindows()
        ActionGroups().unload()

    def __init__(self):

        # Finite State Machine
        self.states = [
            {'name': 'idle'},
            {'name': 'detecting the blue pet door'},
            {'name': 'passing thru the blue pet door'},
            {'name': 'detecting the yellow demarcation line'},
            {'name': 'climbing the curb'},
            {'name': 'on the plate detecting the yellow demarcation line'},
            {'name': 'descending the curb'},
            {'name': 'detecting the black cross'},
        ]
        self.transitions = [
            {'trigger':'start', 'source':'idle', 'dest':'detecting the blue pet door', 'prepare':['setup', 'buffer_frames', 'print_state'], 'before':'detect_the_blue_pet_door', 'after':'follow_the_30mm_black_line'},
            {'trigger':'close_to_the_door', 'source':'detecting the blue pet door', 'dest':'passing thru the blue pet door', 'before':'pass_thru_the_blue_pet_door'},
            {'trigger':'thru_the_door', 'source':'passing thru the blue pet door', 'dest':'detecting the yellow demarcation line', 'before':'detect_the_yellow_demarcation_line'},
            {'trigger':'close_to_the_curb', 'source':'detecting the yellow demarcation line', 'dest':'climbing the curb', 'prepare':'stop_line_following', 'after':'climb_the_curb'},
            {'trigger':'climbed_the_curb', 'source':'climbing the curb', 'dest':'on the plate detecting the yellow demarcation line', 'before':'detect_the_yellow_demarcation_line', 'after':'follow_the_30mm_black_line'},
            {'trigger':'close_to_the_curb', 'source':'on the plate detecting the yellow demarcation line', 'dest':'descending the curb', 'prepare':'stop_line_following', 'after':'descend_the_curb'},
            {'trigger':'descended_the_curb', 'source':'descending the curb', 'dest':'detecting the black cross', 'before':'detect_the_black_cross', 'after':'follow_the_30mm_black_line'},
            {'trigger':'crossed_the_finish_line', 'source':'detecting the black cross', 'dest':'idle', 'before':'finish'},
        ]
        Machine.__init__(self, states=self.states, transitions=self.transitions, initial='idle')

        self.__gait = PUPPY(setServoPulse=setBusServoPulse, servoParams=BusServoParams())  # TODO: 自己设计 gait

        self.line_following_stopped = Event()
        self.finished = Event()

    class submit_to_the_pool:
        def __init__(self, func):
            self.func = func

        def __call__(self, obj, *args, **kwargs):
            obj._pool.submit(self.func, obj, *args, **kwargs)

        def __get__(self, obj, cls):
            if obj is None:
                return self  # Accessed from class, return unchanged
            return types.MethodType(self, obj)  # Accessed from instance, bind to instance

    @submit_to_the_pool
    def setup(self):  # TODO: 自己设计 gait
        self.__gait.stance_config(self._stance(0, 0, -15, 2), pitch=0, roll=0)  # 标准站姿
        self.__gait.gait_config(overlap_time=0.1, swing_time=0.15, z_clearance=3)
        self.__gait.run()  # 启动

        self.__camera.set_pwm_servo_pulse(1600)

    @submit_to_the_pool
    def print_state(self):
        while not self.finished.is_set():
            print(self.state)
            sleep(3)

    @submit_to_the_pool
    def buffer_frames(self):
        while not self.finished.is_set():
            ret, frame = self.__cap.read()
            if not ret:
                continue
            self.frames.append(frame)

    @submit_to_the_pool
    def stop_line_following(self):
        self.line_following_stopped.set()

    @submit_to_the_pool
    def follow_the_30mm_black_line(self):  # TODO: 自己设计 gait
        self.line_following_stopped.clear()
        while not self.finished.is_set() and not self.line_following_stopped.is_set():

            sleep(0.1)

            try:
                frame = self.frames.pop()
            except IndexError:
                continue

            try:
                if self.__camera.pulsewidth is None:
                    continue
                if self.__camera.pulsewidth >= 2200:
                    px_deviation = row_of_pixels_method(frame, 300)
                else:
                    px_deviation = row_of_pixels_method(frame, 479)
            except RuntimeError:
                continue

            if abs(px_deviation) < 60:
                self.__gait.move(x=12, y=0, yaw_rate=0)          # go forward
            if px_deviation >= 60:
                self.__gait.move(x=7, y=0, yaw_rate=-25 / 57.3)  # turn right
            if px_deviation <= -60:
                self.__gait.move(x=7, y=0, yaw_rate=25 / 57.3)   # turn left

        self.__gait.move_stop()

    @submit_to_the_pool
    def detect_the_blue_pet_door(self):
        shape = cv.cvtColor(cv.imread("object_shapes/pet_door.jpg"), cv.COLOR_BGR2GRAY)
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
    def pass_thru_the_blue_pet_door(self):  # TODO: 自己设计 gait
        sleep(2.5)
        self.__gait.stance_config(self._stance(0, 0, -12, 2), pitch=0, roll=0)  # 趴下
        self.__gait.gait_config(overlap_time=0.1, swing_time=0.15, z_clearance=2)
        sleep(8)
        self.__gait.stance_config(self._stance(0, 0, -15, 2), pitch=0, roll=0)  # 过完门站起来.

        self.__camera.set_pwm_servo_pulse(2300)

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

            if area > 160000:
                break
        self.close_to_the_curb()

    @submit_to_the_pool
    def climb_the_curb(self):  # TODO: 自己设计动作
        runActionGroup('coord_up_stair_1')
        self.__gait.stance_config(self._stance(0, 0, -13, -4), pitch=-20 / 57.3, roll=0)
        self.__gait.gait_config(overlap_time=0.2, swing_time=0.4, z_clearance=2)
        self.__gait.move_stop()
        sleep(0.2)
        self.__gait.move(x=2.5, y=0, yaw_rate=0)
        sleep(4)
        self.__gait.move_stop()
        runActionGroup('coord_up_stair_2')
        self.__gait.stance_config(self._stance(0, 0, -13, 0), pitch=0, roll=0)
        self.__gait.gait_config(overlap_time=0.1, swing_time=0.2, z_clearance=1.5)
        self.__gait.move_stop()
        sleep(0.2)

        self.climbed_the_curb()

    @submit_to_the_pool
    def descend_the_curb(self):  # TODO: 自己设计动作

        self.follow_the_30mm_black_line()
        sleep(1)
        self.line_following_stopped.set()

        runActionGroup('coord_down_stair')
        self.__gait.stance_config(self._stance(0, 0, -13, 4), pitch=20 / 57.3, roll=0)
        self.__gait.gait_config(overlap_time=0.1, swing_time=0.2, z_clearance=2)
        self.__gait.move_stop()
        sleep(0.2)
        self.__gait.move(x=3.7, y=0, yaw_rate=0)
        sleep(4)
        self.__gait.stance_config(self._stance(0, 0, -15, 2))
        self.__gait.gait_config(overlap_time=0.1, swing_time=0.15, z_clearance=3)
        self.__gait.move_stop()
        sleep(0.2)

        self.descended_the_curb()

    @submit_to_the_pool
    def detect_the_black_cross(self):
        shape = cv.cvtColor(cv.imread("object_shapes/cross.png"), cv.COLOR_BGR2GRAY)
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

            if disparity < 0.003 and area > 120000:
                break
        self.crossed_the_finish_line()

    @submit_to_the_pool
    def finish(self):
        sleep(5)  # crossed finish line after 5s
        self.finished.set()

    @staticmethod
    def _stance(x=0, y=0, z=-15, x_shift=2):  # 单位cm  # TODO: 自己设计 gait
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

'''