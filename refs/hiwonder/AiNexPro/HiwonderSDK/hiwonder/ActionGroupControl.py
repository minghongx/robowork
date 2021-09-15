#!/usr/bin/env python3
# encoding: utf-8
import os
import time
from . import Board
import sqlite3 as sql

#上位机编辑的动作调用库

runningAction = False
stop_action = False

def stopAction():
    global stop_action
    
    stop_action = True

def runAction(actNum, lock_servos='', path="/home/pi/AiNexPro/ActionGroups/", send_again=True):
    '''
    运行动作组，无法发送stop停止信号
    :param actNum: 动作组名字 ， 字符串类型
    :param send_again:  如果动作运行时间较小，小于200ms，则需设为False
    :return:
    '''
    global runningAction
    global stop_action
    
    if actNum is None:
        return

    actNum = path + actNum + ".d6a"
    #print(actNum)
    if os.path.exists(actNum) is True:
        if runningAction is False:
            runningAction = True
            ag = sql.connect(actNum)
            cu = ag.cursor()
            cu.execute("select * from ActionGroup")
            while True:
                act = cu.fetchone()
                if stop_action is True:
                    stop_action = False
                    print('stop')                    
                    break
                if act is not None:
                    for i in range(0, len(act) - 2, 1):
                        if str(i + 1) in lock_servos:
                            Board.setBusServoPulse(i + 1, lock_servos[str(i + 1)], act[1], send_again)
                        else:
                            Board.setBusServoPulse(i + 1, act[2 + i], act[1], send_again)
                    time.sleep(float(act[1])/1000.0)
                else:   # 运行完才退出
                    break
            runningAction = False
            
            cu.close()
            ag.close()
    else:
        runningAction = False
        print("未能找到动作组文件")
    
if __name__ == '__main__':
    runAction('go_forward')
