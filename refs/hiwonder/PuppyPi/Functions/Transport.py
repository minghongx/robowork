#!/usr/bin/python3
# coding=utf8
import sys
import import_path
import kinematics as kinematics
import HiwonderSDK.Board as Board

HWSONAR = None
ik = kinematics.IK()

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

# 初始位置
def initMove():
    HWSONAR.setRGBMode(0)
    HWSONAR.setRGB(1, (0, 0, 0))
    HWSONAR.setRGB(2, (0, 0, 0))
    Board.setPWMServoPulse(1, 1500, 500)
    Board.setPWMServoPulse(2, 1500, 500)

def reset():
    return None

def init():
    initMove()
    print("Transport Init")
    return None

def start():
    print("Transport Start")
    return None

def stop():
    print("Transport Stop")
    return None

def exit():
    print("Transport Exit")
    ik.stand(ik.initial_pos, t=1000)
    return None

def run(img):
    return img
