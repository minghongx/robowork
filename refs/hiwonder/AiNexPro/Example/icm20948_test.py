#!/usr/bin/python3
# -*- coding: UTF-8 -*-
############9轴传感器测试#############
import os
import sys
import math
import time
import numpy as np
from icm20948 import ICM20948

if not os.path.exists('icm20948_calibration_param.npz'):
    print('没有检测到imu校正保存的数据，请确保已经进行过imu校正')
    sys.exit()

# 加载参数
param_data = np.load('icm20948_calibration_param.npz')

# 获取参数
amin = param_data['amin']
amax = param_data['amax']

imu = ICM20948()

X = 0
Y = 1
Z = 2

AXES = X, Z

while True:
    mag = list(imu.read_magnetometer_data())
    for i in range(3):
        mag[i] -= amin[i]
        try:
            mag[i] /= amax[i] - amin[i]
        except ZeroDivisionError:
            pass
        mag[i] -= 0.5

    heading = math.atan2(
            mag[AXES[0]],
            mag[AXES[1]])

    if heading < 0:
        heading += 2 * math.pi
    heading = math.degrees(heading)
    heading = round(heading)

    print("Heading: {}".format(heading))

    time.sleep(0.1)
