#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import numpy as np
import kinematics  # 运动学库已加密，只供调用

ik = kinematics.IK()  # 实例化逆运动学库
ik.stand(ik.initial_pos, t=500)  # 立正，姿态为ik.initial_pos，时间500ms
print('姿态:\n', np.array(ik.initial_pos))  # 打印查看姿态
# 姿态数据为3x6的数组，表示6条腿的的末端的x，y，z坐标，单位mm
# 头部朝前的方向为x轴， 头部朝前位置为负方向，右边为y轴正， 竖直朝上为z轴正， 从中间两条腿的连线的中点做垂线与上下板相交，取连线中心为零点
# 第一条腿表示头部朝前时左上角所在腿, 逆时针表示1-6
# [[-199.53, -177.73, -100.0],
#  [0.0,     -211.27, -100.0],
#  [199.53,  -177.73, -100.0],
#  [199.53,  177.73,  -100.0],
#  [0.0,     211.27,  -100.0],
#  [-199.53, 177.73,  -100.0]]

# 参数1：姿态；参数2：模式，2为六足模式，4为四足模式，当为4足模式时相应的姿态需要为initial_pos_quadruped
# 参数3：步幅，单位mm （转弯时为角度，单位度）；参数4：速度，单位mm/s；参数5：执行次数，单位0时表示无限循环

ik.go_forward(ik.initial_pos, 2, 50, 80, 1)  # 朝前直走50mm

ik.back(ik.initial_pos, 2, 100, 80, 1)  # 朝后直走100mm

ik.turn_left(ik.initial_pos, 2, 30, 100, 1)  # 原地左转30度

ik.turn_right(ik.initial_pos, 2, 30, 100, 1)  # 原地右转30读

ik.left_move(ik.initial_pos, 2, 100, 100, 1)  # 左移100mm

ik.right_move(ik.initial_pos, 2, 100, 100, 1)  # 右移100mm

ik.stand(ik.initial_pos, t=500)
