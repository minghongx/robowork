import sched
import copy

import numpy as np
import pandas as pd

from puppypi_pro.hiwonder_puppypipro_servo import *


class ActionGroups:

    def __init__(self, data_file_path: str = None):
        self.__action_data = pd.read_csv(data_file_path, dtype={'start_moment': np.uint16,
                                                                'serial_bus_servo_id': np.uint8,
                                                                'pulse': np.uint16,
                                                                'rotating_time': np.uint16}) \
                               .sort_values(by=['start_moment']) if data_file_path else \
                             pd.DataFrame(columns=['start_moment', 'serial_bus_servo_id', 'pulse', 'rotating_time'])
        self.__scheduler = sched.scheduler()

    def add_action(self, start_moment: int, serial_bus_servo_id: int, pulse: int, rotating_time: int = 0):
        self.__action_data = self.__action_data.append({'start_moment': start_moment,
                                                        'serial_bus_servo_id': serial_bus_servo_id,
                                                        'pulse': pulse,
                                                        'rotating_time': rotating_time}, ignore_index=True)\
                                               .sort_values(by=['start_moment'])

    def persist(self, data_file_path: str):
        self.__action_data.to_csv(data_file_path, index=False)

    def do(self):
        if not self.__scheduler.empty():  # 还有动作没有执行
            return
        self._schedule()
        self.__scheduler.run()

    def infinitely_do(self):
        if not self.__scheduler.empty():  # 还有动作没有执行
            return
        self._schedule()
        while True:
            if self.__scheduler.empty():
                break
            snapshot = copy.deepcopy(self.__scheduler)  # FIXME: .stop() 后不能马上停止
            snapshot.run()

    def _schedule(self):

        if self.__action_data.empty:  # 没有动作可以调度
            return

        # Actions
        [self.__scheduler.enter(action['start_moment'] / 1000,
                                1,
                                set_serial_bus_servo_pulse_and_rotating_duration,
                                (action['serial_bus_servo_id'], action['pulse'], action['rotating_time'])
                                ) for _, action in self.__action_data.iterrows()]

        # FIXME: None 会导致持续阻塞
        # 阻塞到动作全部运行完, 而不是阻塞到事件调度完
        # end_of_action_group = self.__action_data.apply(lambda row: row['start_moment'] + row['rotating_time'], axis=1).max()
        # self.__scheduler.enter(end_of_action_group, 1, None)

    def stop(self):
        if self.__scheduler.empty():
            return
        map(self.__scheduler.cancel, self.__scheduler.queue)

    @classmethod
    def unload(cls):  # FIXME: 由于 servo ID 可以设定, 安全的逻辑应该是先获取 12 个 servo ID 再掉电, 而不是 assume servo ID in range(1, 13)
        [unload_serial_bus_servo(servo_id) for servo_id in range(1, 13)]

    @property
    def action_data(self):
        return self.__action_data


'''
Ref
Adverbs of manner tell us "how" something happens.
It is possible to place the adverb before the verb. This places emphasis on the adverb.
https://www.bbc.co.uk/worldservice/learningenglish/flatmates/episode75/languagepoint.shtml
https://docs.python.org/3/library/sched.html
https://docs.python.org/3/library/copy.html
'''
