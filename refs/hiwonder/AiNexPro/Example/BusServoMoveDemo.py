import time
from hiwonder import Board

print('''
**********************************************************
**************功能:幻尔科技串口舵机控制例程***************
**********************************************************
----------------------------------------------------------
Official website:http://www.hiwonder.com
Online mall:https://huaner.tmall.com/
----------------------------------------------------------
以下指令均需在LX终端使用，LX终端可通过ctrl+alt+t打开，或点
击上栏的黑色LX终端图标。
----------------------------------------------------------
Usage:
    sudo python3 BusServoMove.py
----------------------------------------------------------
Version: --V1.1  2021/03/16
----------------------------------------------------------
Tips:
 * 按下Ctrl+C可关闭此次程序运行，若失败请多次尝试！
----------------------------------------------------------
''')

while True:
    # 参数：参数1：舵机id; 参数2：位置; 参数3：运行时间
    Board.setBusServoPulse(19, 500, 500) # 19号舵机转到500位置，用时500ms
    time.sleep(0.5) # 延时时间和运行时间相同
    
    Board.setBusServoPulse(19, 200, 500) #舵机的转动范围0-240度，对应的脉宽为0-1000,即参数2的范围为0-1000
    time.sleep(0.5)
    
    Board.setBusServoPulse(19, 500, 200)
    time.sleep(0.2)
    
    Board.setBusServoPulse(19, 200, 500)  
    Board.setBusServoPulse(20, 300, 500)
    time.sleep(0.5)
    
    Board.setBusServoPulse(19, 500, 500)  
    Board.setBusServoPulse(20, 500, 500)
    time.sleep(0.5)    
