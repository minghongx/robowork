#!/usr/bin/python3
# -*- coding: UTF-8 -*-
##############9轴传感器校正###############
import signal
import numpy as np
from icm20948 import ICM20948

print("""旋转机器人360度来获取最大，最小值， 通过nump保存以供调用""")

imu = ICM20948()

amin = list(imu.read_magnetometer_data())
amax = list(imu.read_magnetometer_data())

Running = True
#暂停信号的回调
def Stop(signum, frame):
    global amin, amax, Running
    
    Running = False
    amin = np.array(amin)
    amax = np.array(amax)
    np.savez('icm20948_calibration_param', amin=amin, amax=amax, fmd='%d', delimiter=' ')
    # 加载参数
    param_data = np.load('icm20948_calibration_param.npz')

    # 获取参数
    amin = param_data['amin']
    amax = param_data['amax']
    print('保存数据：',amin, amax)

signal.signal(signal.SIGINT, Stop)

while True:
    if Running:
        mag = list(imu.read_magnetometer_data())
        for i in range(3):
            v = mag[i]
            if v < amin[i]:
                amin[i] = v
            if v > amax[i]:
                amax[i] = v
        print('\namin:', amin, '\namax:', amax)
    else:
        break
