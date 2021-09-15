#!/usr/bin/python3
# coding=utf8
import sys
import import_path
import HiwonderSDK.Board as Board

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

HWSONAR = None

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
    print("RemoteControl Init")
    return None

def start():
    print("RemoteControl Start")
    return None

def stop():
    print("RemoteControl Stop")
    return None

def exit():
    print("RemoteControl Exit")
    return None

def run(img):
    return img
