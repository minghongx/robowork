#!/usr/bin/python3
#coding=utf8
import sys
import time
import import_path
import kinematics as kinematics

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

ik = kinematics.IK()

X1, Y1 = 119.53, 57.73  # mm
X2, Y2 = 0.00, 91.27  # mm

pos_1 = [
    [-80 - X1, -120 - Y1, -100],
    [-0 -  X2, -120 - Y2, -80],
    [80 +  X1, -120 - Y1, -100],
    [80 +  X1, 120 +  Y1, -100],
    [-0 +  X2, 120 +  Y2, -80],
    [-80 - X1, 120 +  Y1, -100]
]

pos_2 = [
    [-80 -  X1, -120 - Y1, -100],
    [-100 - X2, -60 -  Y2, -80],
    [80 +   X1, -120 - Y1, -100],
    [80 +   X1, 120 +  Y1, -100],
    [-100 + X2, 60 +   Y2, -80],
    [-80 -  X1, 120 +  Y1, -100]
]

pos_3 = [
    [-80 -  X1, -120 - Y1, -100],
    [-100 - X2, -60 -  Y2, -100],
    [80 +   X1, -120 - Y1, -100],
    [80 +   X1, 120 +  Y1, -100],
    [-100 + X2, 60 +   Y2, -100],
    [-80 -  X1, 120 +  Y1, -100]
]

pos_4 = [
    [-80 -  X1, -120 - Y1, -100],
    [-100 - X2, -60 -  Y2, -100],
    [-0 +   X1, -120 - Y1, -100],
    [-0 +   X1, 120 +  Y1, -100],
    [-100 + X2, 60  +  Y2, -100],
    [-80 -  X1, 120 +  Y1, -100]
]

def pick(pick=True, mode=1):
    if pick:
        if mode == 1:
            ik.stand(ik.initial_pos)
            time.sleep(0.5)
            ik.stand(pos_1, t=500)
            ik.stand(pos_2, t=1000)
            ik.stand(pos_3, t=500)
            ik.stand(pos_4, t=1000)
            time.sleep(0.5)       
        for i in range(2, 6):
            transport(i)
    else:
        ik.stand(ik.initial_pos_quadruped)       
        for i in range(6, 0, -1):
            transport(i)
        if mode == 1:
            time.sleep(0.5)
            ik.stand(pos_4, t=1000)
            ik.stand(pos_3, t=1000)
            ik.stand(pos_2, t=500)
            ik.stand(pos_1, t=1000)
            ik.stand(ik.initial_pos)

def transport(step):
    if step == 1:
        # 1
        initial_pos = [
            [-20 -  X1, 0 -    Y1, 210],
            [-120 - X2, -30 -  Y2, -100],
            [-0 +   X1, -120 - Y1, -100],
            [-0 +   X1, 120 +  Y1, -100],
            [-120 + X2, 30 +   Y2, -100],
            [-20 -  X1, 0 +    Y1, 210]
        ]
        ik.stand(initial_pos, t=1000)
    elif step == 2:
        # 2
        initial_pos = [
            [-220 - X1, -90 -  Y1, -40],
            [-120 - X2, -60 -  Y2, -70],
            [-0 +   X1, -120 - Y1, -70],
            [-0 +   X1, 120 +  Y1, -70],
            [-120 + X2, 60 +   Y2, -70],
            [-220 - X1, 90 +   Y1, -40]
        ]
        ik.stand(initial_pos, t=1000)
        time.sleep(0.5)
    elif step == 3:
        # 3
        initial_pos = [
            [-220 - X1, -0 -   Y1, -40],
            [-120 - X2, -60 -  Y2, -70],
            [-0 +   X1, -120 - Y1, -70],
            [-0 +   X1, 120 +  Y1, -70],
            [-120 + X2, 60 +   Y2, -70],
            [-220 - X1, 0 +    Y1, -40]
        ]
        ik.stand(initial_pos, t=1000)
        time.sleep(0.5)
    elif step == 4:
        # 4
        initial_pos = [
            [-20 -  X1, 0 -    Y1, 210],
            [-120 - X2, -60 -  Y2, -100],
            [-0 +   X1, -120 - Y1, -100],
            [-0 +   X1, 120 +  Y1, -100],
            [-120 + X2, 60 +   Y2, -100],
            [-20 -  X1, 0 +    Y1, 210]
        ]
        ik.stand(initial_pos, t=1000)
        time.sleep(0.5)
    elif step == 5:
        # 5
        initial_pos = [
            [-20 -  X1, 0 -    Y1, 210],
            [-120 - X2, -30 -  Y2, -100],
            [-0 +   X1, -120 - Y1, -100],
            [-0 +   X1, 120 +  Y1, -100],
            [-120 + X2, 30 +   Y2, -100],
            [-20 -  X1, 0 +    Y1, 210]
        ]
        ik.stand(initial_pos)

if __name__ == '__main__':
    pick(True)
    pick(False)
