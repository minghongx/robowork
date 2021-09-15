import time
import Board

print('''
**********************************************************
********功能:幻尔科技SpiderPi扩展板，串口舵机控制例程*******
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
Version: --V1.1  2020/11/13
----------------------------------------------------------
Tips:
 * 按下Ctrl+C可关闭此次程序运行，若失败请多次尝试！
----------------------------------------------------------
''')

while True:
    # 参数：参数1：舵机id; 参数2：位置; 参数3：运行时间
    Board.setBusServoPulse(9, 200, 1000)  
    Board.setBusServoPulse(7, 500, 1000)
    time.sleep(1)
    
    Board.setBusServoPulse(9, 500, 500)  # 9号舵机转到500位置，用时500ms
    time.sleep(0.5)  # 延时时间和运行时间相同
    
    Board.setBusServoPulse(9, 800, 500)  #舵机的转动范围0-240度，对应的脉宽为0-1000,即参数2的范围为0-1000
    time.sleep(0.5)
    
    Board.setBusServoPulse(7, 300, 400)
    time.sleep(0.4)

    Board.setBusServoPulse(7, 700, 800)
    time.sleep(0.8) 

    Board.setBusServoPulse(7, 500, 400)
    time.sleep(0.4)  

   
