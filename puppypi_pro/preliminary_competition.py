import functools
# from operator import methodcaller
from abc import ABC, abstractmethod
from concurrent.futures import ThreadPoolExecutor
from collections import deque
from multiprocessing import Event
from time import sleep
from pathlib import Path

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

        # TODO: encapsulate them into a Camera Class
        self.__cap = cv.VideoCapture(-1)
        self.__camera = PWMServos(12)
        self.frames = deque(maxlen=2)

        self._pool = ThreadPoolExecutor(max_workers=8)

        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.finished.set()  # 确保意外退出 with-statement 时带有循环体的线程能全部结束并最终回收资源
        self._pool.shutdown()
        self.__cap.release()
        cv.destroyAllWindows()
        ActionGroups().unload()

    def __init__(self):

        # Finite State Machine
        self.states = [
            {'name': 'idle'},
            {'name': 'ready'},
            {'name': 'detecting the blue pet door'},
            {'name': 'passing thru the blue pet door'},
            {'name': 'detecting the yellow demarcation line'},
            {'name': 'climbing the curb'},
            {'name': 'on the plate detecting the yellow demarcation line'},
            {'name': 'descending the curb'},
            {'name': 'detecting the black cross'},
        ]
        self.transitions = [
            {'trigger':'setup', 'source':'idle', 'dest':'ready', 'prepare':'initiate', 'before':'buffer_frames'},
            {'trigger':'start', 'source':'ready', 'dest':'detecting the blue pet door', 'prepare':'print_state', 'before':'detect_the_blue_pet_door', 'after':'follow_the_30mm_black_line'},
            {'trigger':'close_to_the_door', 'source':'detecting the blue pet door', 'dest':'passing thru the blue pet door', 'before':'pass_thru_the_blue_pet_door'},
            {'trigger':'thru_the_door', 'source':'passing thru the blue pet door', 'dest':'detecting the yellow demarcation line', 'before':'detect_the_yellow_demarcation_line'},
            {'trigger':'close_to_the_curb', 'source':'detecting the yellow demarcation line', 'dest':'climbing the curb', 'prepare':'stop_line_following', 'after':'climb_the_curb'},
            {'trigger':'climbed_the_curb', 'source':'climbing the curb', 'dest':'on the plate detecting the yellow demarcation line', 'before':'detect_the_yellow_demarcation_line', 'after':'follow_the_30mm_black_line'},
            {'trigger':'close_to_the_curb', 'source':'on the plate detecting the yellow demarcation line', 'dest':'descending the curb', 'prepare':'stop_line_following', 'after':'descend_the_curb'},
            {'trigger':'descended_the_curb', 'source':'descending the curb', 'dest':'detecting the black cross', 'before':'detect_the_black_cross', 'after':'follow_the_30mm_black_line'},
            {'trigger':'crossed_the_finish_line', 'source':'detecting the black cross', 'dest':'idle', 'before':'finish'},
        ]
        Machine.__init__(self, states=self.states, transitions=self.transitions, initial='idle', send_event=True)

        self.__gait = PUPPY(setServoPulse=setBusServoPulse, servoParams=BusServoParams())  # TODO: 自己设计 gait

        self.line_following_stopped = Event()
        self.finished = Event()

    def submit_to_the_pool(done_callback: str = None):
        def decorator(func):
            @functools.wraps(func)
            def wrapper(obj, *args, **kwargs):
                if done_callback is None:
                    obj._pool.submit(func, obj, *args, **kwargs)
                else:
                    callback = getattr(obj, done_callback)
                    obj._pool.submit(func, obj, *args, **kwargs).add_done_callback(callback)
            return wrapper
        return decorator

    @submit_to_the_pool()
    def initiate(self, event):  # TODO: 自己设计 gait
        self.__gait.stance_config(self._stance(0, 0, -15, 2), pitch=0, roll=0)  # 标准站姿
        self.__gait.gait_config(overlap_time=0.1, swing_time=0.15, z_clearance=3)
        self.__gait.run()  # 启动

        self.__camera.set_pwm_servo_pulse(1600)

    @submit_to_the_pool()
    def print_state(self, event):
        while not self.finished.is_set():
            print(self.state)
            sleep(3)

    @submit_to_the_pool()
    def buffer_frames(self, event):
        while not self.finished.is_set():
            ret, frame = self.__cap.read()
            if not ret:
                continue
            self.frames.append(frame)

    @submit_to_the_pool()
    def stop_line_following(self, event):
        self.line_following_stopped.set()

    @submit_to_the_pool()
    def follow_the_30mm_black_line(self, event):  # TODO: 自己设计 gait
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

    @submit_to_the_pool(done_callback='close_to_the_door')
    def detect_the_blue_pet_door(self, event):
        shape = cv.imread(str(Path(__file__).parent/'object_shapes/pet_door.jpg'), cv.IMREAD_GRAYSCALE)  # The image should be in the working directory or a full path of image should be given.
        while not self.finished.is_set():

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

    @submit_to_the_pool(done_callback='thru_the_door')
    def pass_thru_the_blue_pet_door(self, event):  # TODO: 自己设计 gait
        sleep(2.5)  # FIXME: 判断非常接近门的愚蠢标准
        self.__gait.stance_config(self._stance(0, 0, -12, 2), pitch=0, roll=0)  # 趴下
        self.__gait.gait_config(overlap_time=0.1, swing_time=0.15, z_clearance=2)
        sleep(8)  # FIXME: 判断全身都通过门的愚蠢标准
        self.__gait.stance_config(self._stance(0, 0, -15, 2), pitch=0, roll=0)  # 过完门站起来.

        self.__camera.set_pwm_servo_pulse(2300)

    @submit_to_the_pool(done_callback='close_to_the_curb')
    def detect_the_yellow_demarcation_line(self, event):
        while not self.finished.is_set():

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

    @submit_to_the_pool(done_callback='climbed_the_curb')
    def climb_the_curb(self, event):  # TODO: 自己设计动作
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

    @submit_to_the_pool(done_callback='descended_the_curb')
    def descend_the_curb(self, event):  # TODO: 自己设计动作

        # FIXME: 因为判断是否足够接近 curb 的标准不够好，所以只能勉强机器重新巡线向前走 1s，再开始下 curb。
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

    @submit_to_the_pool(done_callback='crossed_the_finish_line')
    def detect_the_black_cross(self, event):
        shape = cv.imread(str(Path(__file__).parent/'object_shapes/cross.png'), cv.IMREAD_GRAYSCALE)
        while not self.finished.is_set():

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

    @submit_to_the_pool()
    def finish(self, event):
        sleep(5)  # FIXME: 判断全身都通过终点线的愚蠢标准
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