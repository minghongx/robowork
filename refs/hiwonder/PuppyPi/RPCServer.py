#!/usr/bin/python3
# coding=utf8
import os
import sys
import time
import math
import copy
import logging
import threading
from werkzeug.serving import run_simple
from werkzeug.wrappers import Request, Response
from jsonrpc import JSONRPCResponseManager, dispatcher

from ActionGroupDict import action_group_dict

import HiwonderSDK.Board as Board
import HiwonderSDK.Mpu6050 as Mpu6050
import HiwonderSDK.ActionGroupControl as AGC

import Functions.PickAction as Pick
import Functions.kinematics as kinematics

# 远程调用api，框架jsonrpc
# 主要用于手机端和电脑端的客户端调用

HWSONAR = None
QUEUE = None

Avoidance = None
Transport = None
FaceDetect = None
ColorTrack = None
ColorDetect = None
VisualPatrol = None
RemoteControl = None
ApriltagDetect = None

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

__RPC_E01 = "E01 - Invalid number of parameter!"
__RPC_E02 = "E02 - Invalid parameter!"
__RPC_E03 = "E03 - Operation failed!"
__RPC_E04 = "E04 - Operation timeout!"
__RPC_E05 = "E05 - Not callable"

Board.setBuzzer(1)
time.sleep(0.1)
Board.setBuzzer(0)

ik = kinematics.IK()
ik.stand(ik.initial_pos)

#mpu6050初始化
mpu = Mpu6050.mpu6050(0x68)
mpu.set_gyro_range(mpu.GYRO_RANGE_2000DEG)
mpu.set_accel_range(mpu.ACCEL_RANGE_2G)

@dispatcher.add_method
def SetPWMServo(*args, **kwargs):
    ret = (True, ())
#     print(args)
    arglen = len(args)
    if 0 != (arglen % 2):
        return (False, __RPC_E01)
    try:
        servos = args[2:arglen:2]
        pulses = args[3:arglen:2]
        use_times = args[0]
        for s in servos:
            if s < 1 or s > 2:
                return (False, __RPC_E02)
        dat = zip(servos, pulses)
        for (s, p) in dat:
            Board.setPWMServoPulse(s, p, use_times)
    except Exception as e:
        print(e)
        ret = (False, __RPC_E03)
    return ret

@dispatcher.add_method
def SetBusServoPulse(*args, **kwargs):
    ret = (True, ())   
    arglen = len(args)
    if (args[1] * 2 + 2) != arglen or arglen < 4:
        return (False, __RPC_E01)
    try:
        servos = args[2:arglen:2]
        pulses = args[3:arglen:2]
        use_times = args[0]
        for s in servos:
           if s < 1 or s > 16:
                return (False, __RPC_E02)
        dat = zip(servos, pulses)
        for (s, p) in dat:
            Board.setBusServoPulse(s, p, use_times)
    except Exception as e:
        print(e)
        ret = (False, __RPC_E03)
    return ret

@dispatcher.add_method
def SetBusServoDeviation(*args):
    ret = (True, ())
    arglen = len(args)
    if arglen != 2:
        return (False, __RPC_E01)
    try:
        servo = args[0]
        deviation = args[1]
        Board.setBusServoDeviation(servo, deviation)
    except Exception as e:
        print(e)
        ret = (False, __RPC_E03)
    return ret

@dispatcher.add_method
def GetBusServosDeviation(args):
    ret = (True, ())
    data = []
    if args != "readDeviation":
        return (False, __RPC_E01)
    try:
        for i in range(1, 7):
            dev = Board.getBusServoDeviation(i)
            if dev is None:
                dev = 999
            data.append(dev)
        ret = (True, data)
    except Exception as e:
        print(e)
        ret = (False, __RPC_E03)
    return ret 

@dispatcher.add_method
def SaveBusServosDeviation(args):
    ret = (True, ())
    if args != "downloadDeviation":
        return (False, __RPC_E01)
    try:
        for i in range(1, 7):
            dev = Board.saveBusServoDeviation(i)
    except Exception as e:
        print(e)
        ret = (False, __RPC_E03)
    return ret 

@dispatcher.add_method
def UnloadBusServo(args):
    ret = (True, ())
    if args != 'servoPowerDown':
        return (False, __RPC_E01)
    try:
        for i in range(1, 7):
            Board.unloadBusServo(i)
    except Exception as e:
        print(e)
        ret = (False, __RPC_E03)
    return ret

@dispatcher.add_method
def GetBusServosPulse(args):
    ret = (True, ())
    data = []
    if args != 'angularReadback':
        return (False, __RPC_E01)
    try:
        for i in range(1, 7):
            pulse = Board.getBusServoPulse(i)
            if pulse is None:
                ret = (False, __RPC_E04)
                return ret
            else:
                data.append(pulse)
        ret = (True, data)
    except Exception as e:
        print(e)
        ret = (False, __RPC_E03)
    return ret 

@dispatcher.add_method
def StopActionGroup():
    ret = (True, ())
    try:     
        AGC.stopAction()
    except Exception as e:
        print(e)
        ret = (False, __RPC_E03)
    return ret

servo_move = False

th1 = None
th2 = None
th3 = None
th4 = None
have_move = True
control_lock = False
@dispatcher.add_method
def Move(*args):
    global th1
    global have_move
    
    ret = (True, ()) 

    #print('args:', args)
    mode = int(args[0])
    movement_direction = float(args[1])
    rotation = float(args[2])
    #speed = speed = float(1000 * (10 / int(400)))
    speed = float(1000 * (8 / int(args[3])))
    times = int(args[4])
    
    if th2 is not None:
        if th2.is_alive():
            return ret
    if th3 is not None:
        if th3.is_alive():
            return ret
    if th4 is not None:
        if th4.is_alive():
            return ret

    if control_lock:
        return ret

    if len(args) != 5:
        return (False, __RPC_E01)
    try:
        if times == 1:
            if have_move:
                ik.stopMove()
                have_move = False
        else:
            if th1 is None:
                th1 = threading.Thread(target=ik.move, args=(ik.current_pos, mode, 80, movement_direction, rotation, speed, times))
                th1.start()
                have_move = True
            else:
                if not th1.is_alive():
                    th1 = threading.Thread(target=ik.move, args=(ik.current_pos, mode, 80, movement_direction, rotation, speed, times))
                    th1.start()
                    have_move = True

    except Exception as e:
        print(e)
        ret = (False, __RPC_E03)
    return ret

@dispatcher.add_method
def Transport(args):
    global th4
    
    ret = (True, ())
    if th1 is not None:
        if th1.is_alive():
            return ret

    if th4 is None:
        th4 = threading.Thread(target=Pick.pick, args=(int(args), 2))
        th4.start()
    else:
        if not th4.is_alive():
            th4 = threading.Thread(target=Pick.pick, args=(int(args), 2))
            th4.start()
    return ret

@dispatcher.add_method
def Stand(*args):
    global th3
    
    ret = (True, ())
    if th4 is not None:
        if th4.is_alive():
            return ret
    if th2 is not None:
        if th2.is_alive():
            return ret
    if th1 is not None:
        if th1.is_alive():
            return ret
    
    if control_lock:
        return ret
    
    height = int(args[0])
    mode = int(args[1])
    t = int(args[2])
    
    #print(height, mode, t)

    if len(args) != 3:
        return (False, __RPC_E01)
    try:
        if t == 800:
            if mode == 2:
                pos = copy.deepcopy(ik.initial_pos)
            else:
                pos = copy.deepcopy(ik.initial_pos_quadruped)
        else:
            if mode == 2:
                if height > 160:
                    ik.current_pos = copy.deepcopy(ik.initial_pos_high)
                else:
                    ik.current_pos = copy.deepcopy(ik.initial_pos)
            else:
                ik.current_pos = copy.deepcopy(ik.initial_pos_quadruped)
            pos = ik.current_pos

        if mode == 2:
            for i in range(6):
                pos[i][2] = -float(height)
        if th3 is None:
            th3 = threading.Thread(target=ik.stand, args=(pos, t))
            th3.start()
        else:
            if not th3.is_alive():
                th3 = threading.Thread(target=ik.stand, args=(pos, t))
                th3.start()
    except Exception as e:
        print(e)
        ret = (False, __RPC_E03)
    return ret

have_run = True
@dispatcher.add_method
def RunAction(*args):
    global th2
    global have_run

    ret = (True, ())
    #print(args)
    actName = '0'
    times = 1

    if th3 is not None:
        if th3.is_alive():
            return
    if th1 is not None:
        if th1.is_alive():
            return
    
    if control_lock:
        return

    if len(args) != 2:
        return (False, __RPC_E01)
    try:
        if args[0] == '0':
            if have_run:
                AGC.stopAction()
                have_run = False
        else:
            if th2 is not None:
                if not th2.is_alive():
                    if args[0] in action_group_dict:
                        actName = action_group_dict[args[0]]
                    else:
                        actName = args[0]
                    times = int(args[1])
                    th2 = threading.Thread(target=AGC.runActionGroup, args=(actName, times))
                    th2.start()
                    have_run = True
            else:
                if args[0] in action_group_dict:
                    actName = action_group_dict[args[0]]
                else:
                    actName = args[0]
                times = int(args[1])
                th2 = threading.Thread(target=AGC.runActionGroup, args=(actName, times))
                th2.start()
    except Exception as e:
        print(e)
        ret = (False, __RPC_E03)
    return ret

unload = True
stand_up_on = False
def stand_up():
    global control_lock

    count1 = 0
    count2 = 0
    while True:
        if not unload and stand_up_on:
            try:
                accel_date = mpu.get_accel_data(g=True)  # 获取传感器值
                angle_x = int(math.degrees(math.atan2(accel_date['x'], accel_date['z'])))  # 转化为角度值
                if abs(angle_x) > 130:
                    count1 += 1
                else:
                    count1 = 0
                if abs(angle_x) < 50:
                    count2 += 1
                else:
                    count2 = 0
                time.sleep(0.1)
                if count1 >= 3:  #往后倒了一定时间后起来
                    count1 = 0
                    if not control_lock:
                        control_lock = True
                        AGC.runAction('stand_flip')
                elif count2 >= 3:  #后前倒了一定时间后起来
                    count2 = 0
                    if control_lock:
                        control_lock = False
                        ik.stand(ik.current_pos, t=500)
            except BaseException as e:
                print(e)
        else:
            control_lock = False
            count1 = 0
            count2 = 0
            time.sleep(0.01)

threading.Thread(target=stand_up).start()

@dispatcher.add_method
def PostureDetect(args):
    global stand_up_on
    
    ret = (True, ())
    #print(args)
    Board.setBuzzer(1)
    time.sleep(0.05)
    Board.setBuzzer(0)
    stand_up_on = int(args)
    return ret

def runbymainth(req, pas):
    if callable(req):
        event = threading.Event()
        ret = [event, pas, None]
        QUEUE.put((req, ret))
        count = 0
        #ret[2] =  req(pas)
        #print('ret', ret)
        while ret[2] is None:
            time.sleep(0.01)
            count += 1
            if count > 200:
                break
        if ret[2] is not None:
            if ret[2][0]:
                return ret[2]
            else:
                return (False, __RPC_E03 + " " + ret[2][1])
        else:
            return (False, __RPC_E04)
    else:
        return (False, __RPC_E05)

@dispatcher.add_method
def LoadFunc(new_func=0):
    global unload
    global stand_up_on
    global control_lock

    control_lock = False
    if new_func == 1:
        stand_up_on = False
        unload = False
    return runbymainth(Running.loadFunc, (new_func, ))

@dispatcher.add_method
def UnloadFunc():
    global unload
    global control_lock
    
    control_lock = False
    unload = True
    return runbymainth(Running.unloadFunc, ())

@dispatcher.add_method
def StartFunc():
    return runbymainth(Running.startFunc, ())

@dispatcher.add_method
def StopFunc():
    return runbymainth(Running.stopFunc, ())

@dispatcher.add_method
def FinishFunc():
    return runbymainth(Running.finishFunc, ())

@dispatcher.add_method
def Heartbeat():
    return runbymainth(Running.doHeartbeat, ())

@dispatcher.add_method
def GetRunningFunc():
    return (True, (0,))

@dispatcher.add_method
def SetTargetTrackingColor(*target_color):
    return runbymainth(ColorTrack.setTargetColor, target_color)

@dispatcher.add_method
def SetVisualPatrolColor(*target_color):
    return runbymainth(VisualPatrol.setLineTargetColor, target_color)

@dispatcher.add_method
def SetSonarDistanceThreshold(new_threshold=40):
    return runbymainth(Avoidance.setThreshold, (new_threshold,))

@dispatcher.add_method
def SetSonarRGB(index, r, g, b):
    global HWSONAR

    if index == 0:
        HWSONAR.setRGB(1, (r, g, b))
        HWSONAR.setRGB(2, (r, g, b))
    else:
        HWSONAR.setRGB(index, (r, g, b))
    return (True, (r, g, b))

@Request.application
def application(request):
    dispatcher["echo"] = lambda s: s
    dispatcher["add"] = lambda a, b: a + b
    response = JSONRPCResponseManager.handle(request.data, dispatcher)
    
    return Response(response.json, mimetype='application/json')

def startRPCServer():
    run_simple('', 9030, application)

if __name__ == '__main__':
    startRPCServer()
