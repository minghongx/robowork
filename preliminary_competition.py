from concurrent.futures import ProcessPoolExecutor
from multiprocessing import Event, Queue
from abc import ABC, abstractmethod
from time import sleep

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

    def __enter__(self):
        # self.process_pool = mp.Pool(mp.cpu_count())
        self.executor = ProcessPoolExecutor(max_workers=cv.getNumberOfCPUs()-1)
        self.cap = cv.VideoCapture(0)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.cap.release()
        cv.destroyAllWindows()
        self.executor.shutdown()
        # self.process_pool.terminate()
        # self.process_pool.join()

    def __init__(self):

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
            {'trigger':'failed', 'source':'*', 'dest':'idle', 'after':'idle'},
            {'trigger':'start', 'source':'idle', 'dest':'detect the blue pet door', 'before':'buffer_frames', 'after':['follow_the_30mm_black_line', 'detect_the_blue_pet_door']},
            {'trigger':'close_to_the_door', 'source':'detect the blue pet door', 'dest':'pass thru the blue pet door', 'after':'pass_thru_the_blue_pet_door'},
            {'trigger':'thru_the_door', 'source':'pass thru the blue pet door', 'dest':'detect the yellow demarcation line', 'after':'detect_the_yellow_demarcation_line'},
            {'trigger':'close_to_the_curb', 'source':'detect the yellow demarcation line', 'dest':'on the plate detect the yellow demarcation line', 'before':'climb_the_curb', 'after':'detect_the_yellow_demarcation_line'},
            {'trigger':'close_to_the_curb', 'source':'on the plate detect the yellow demarcation line', 'dest':'detect the black cross', 'before':'descend_the_curb', 'after':'detect_the_black_cross'},
            {'trigger':'crossed_the_finish_line', 'source':'detect the black cross', 'dest':'idle', 'after':'idle'},
        ]
        Machine.__init__(self, states=self.states, transitions=self.transitions, initial='idle')

        self.gait = PUPPY(setServoPulse=setBusServoPulse, servoParams=BusServoParams())  # FIXME: 自己设计 gait
        self.camera = PWMServos(12)  # TODO: encapsulate a Camera Class
        self.frames = Queue(5)
        self.start_following_line = Event()

    def idle(self):
        self.start_following_line.clear()  # stop following line
        self.camera.set_pwm_servo_pulse(1500)
        ActionGroups.unload()  # unload servos to save power

    # def prepare

    def buffer_frames(self):
        def buffer_frames():
            while True:
                ret, frame = self.cap.read()
                if ret:
                    continue
                if self.frames.full():
                    self.frames.get()  # imitating a double ended queue
                self.frames.put(frame)
        self.executor.submit(buffer_frames)

    def follow_the_30mm_black_line(self):  # FIXME: 自己设计 gait
        def follow_the_30mm_black_line():
            # FIXME
            camera_position = 1600
            self.camera.set_pwm_servo_pulse(camera_position)
            while True:
                self.start_following_line.wait()

                px_deviation = row_of_pixels_method(self.frames.get(), 420)

                if abs(px_deviation) < 65:
                    self.gait.move(x=12, y=0, yaw_rate=0)  # go forward
                    camera_position -= 27 if camera_position > 1600 else 0
                    self.camera.set_pwm_servo_pulse(camera_position)  # FIXME
                if px_deviation >= 65:
                    self.gait.move(x=7, y=0, yaw_rate=-25 / 57.3)  # turn right
                    camera_position += 22 if camera_position < 2300 else 0
                    self.camera.set_pwm_servo_pulse(camera_position)  # FIXME
                if px_deviation <= -65:
                    self.gait.move(x=7, y=0, yaw_rate=25 / 57.3)  # turn left
                    camera_position += 22 if camera_position < 2300 else 0
                    self.camera.set_pwm_servo_pulse(camera_position)  # FIXME

        self.start_following_line.set()
        self.executor.submit(follow_the_30mm_black_line)

    def detect_the_blue_pet_door(self):
        def detect_the_blue_pet_door():
            while True:
                hsv = cv.cvtColor(self.frames.get(), cv.COLOR_BGR2HSV)
                blurred_hsv = cv.GaussianBlur(hsv, (3, 3), 3)
                blue = cv.inRange(blurred_hsv, (100, 70, 46), (124, 255, 255))
                opened_blue = cv.morphologyEx(blue, cv.MORPH_OPEN, (6, 6))
                contours, _ = cv.findContours(opened_blue, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_TC89_L1)
                try:
                    presumable_door_contour, area = _calc_max_contour_area(contours)
                except ValueError:
                    continue
                x, y, w, h = cv.boundingRect(presumable_door_contour)
                disparity = cv.matchShapes(cv.imread("pet_door_standard.jpg"), opened_blue[y:y+h, x:x+w], cv.CONTOURS_MATCH_I1, 0)
                if disparity < 0.1:
                    break
        self.executor.submit(detect_the_blue_pet_door).add_done_callback(self.close_to_the_door)

    def pass_thru_the_blue_pet_door(self):  # FIXME: 自己设计 gait
        def pass_thru_the_blue_pet_door():
            self.camera.set_pwm_servo_pulse(2100)
            self.gait.stance_config(self._stance(0, 0, -12, 2), pitch=0, roll=0)  # 趴下
            self.gait.gait_config(overlap_time=0.1, swing_time=0.15, z_clearance=2)
            sleep(10)
        self.executor.submit(pass_thru_the_blue_pet_door).add_done_callback(self.thru_the_door)

    def detect_the_yellow_demarcation_line(self):
        def detect_the_yellow_demarcation_line():
            while True:
                hsv = cv.cvtColor(self.frames.get(), cv.COLOR_BGR2HSV)
                blurred_hsv = cv.GaussianBlur(hsv, (3, 3), 3)
                yellow = cv.inRange(blurred_hsv, (20, 44, 44), (38, 255, 255))
                opened_blue = cv.morphologyEx(yellow, cv.MORPH_OPEN, (6, 6))
                contours, _ = cv.findContours(opened_blue, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_TC89_L1)[0]
                try:
                    _, area = _calc_max_contour_area(contours)
                except ValueError:
                    continue
                if area > 40000:
                    break
        self.executor.submit(detect_the_yellow_demarcation_line).add_done_callback(self.close_to_the_curb)

    def climb_the_curb(self):  # FIXME: 自己设计动作
        def climb_the_curb():
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

        self.executor.submit(climb_the_curb)

    def descend_the_curb(self):  # FIXME: 自己设计动作
        def descend_the_curb():
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

        self.executor.submit(descend_the_curb)

    def detect_the_black_cross(self):
        def detect_the_black_cross():
            while True:
                _, otsu_binary_img = cv.threshold(cv.cvtColor(self.frames.get(), cv.COLOR_BGR2GRAY), 0, 255, cv.THRESH_BINARY_INV + cv.THRESH_OTSU)
                opened_otsu_binary_img = cv.morphologyEx(otsu_binary_img, cv.MORPH_OPEN, (7, 7))
                contours, _ = cv.findContours(opened_otsu_binary_img, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_TC89_L1)
                try:
                    presumable_door_contour, area = _calc_max_contour_area(contours)
                except ValueError:
                    continue
                x, y, w, h = cv.boundingRect(presumable_door_contour)
                disparity = cv.matchShapes(cv.imread("cross_standard.png"), opened_otsu_binary_img[y:y+h, x:x+w], cv.CONTOURS_MATCH_I1, 0)
                if disparity < 0.1:
                    break
        self.executor.submit(detect_the_black_cross).add_done_callback(self.crossed_the_finish_line)

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