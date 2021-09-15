import time
import Board

print('''
**********************************************************
*****功能:幻尔科技SpiderPi扩展板，串口舵机读取状态例程******
**********************************************************
----------------------------------------------------------
Official website:http://www.hiwonder.com
Online mall:https://huaner.tmall.com/
----------------------------------------------------------
以下指令均需在LX终端使用，LX终端可通过ctrl+alt+t打开，或点
击上栏的黑色LX终端图标。
----------------------------------------------------------
Usage:
    sudo python3 BusServoReadStatus.py
----------------------------------------------------------
Version: --V1.1  2020/11/13
----------------------------------------------------------
Tips:
 * 按下Ctrl+C可关闭此次程序运行，若失败请多次尝试！
----------------------------------------------------------
''')

def getBusServoStatus():
    Pulse = Board.getBusServoPulse(9)  # 获取9号舵机的位置信息
    Temp = Board.getBusServoTemp(9)  # 获取9号舵机的温度信息
    Vin = Board.getBusServoVin(9)  # 获取9号舵机的电压信息
    print('Pulse: {}\nTemp:  {}℃\nVin:   {}mV\n'.format(Pulse, Temp, Vin)) # 打印状态信息
    time.sleep(0.5)  # 延时方便查看

while True:   
    Board.setBusServoPulse(9, 500, 1000) # 9号舵机转到500位置用时1000ms
    time.sleep(1)
    getBusServoStatus()
    Board.setBusServoPulse(9, 300, 1000)
    time.sleep(1)
    getBusServoStatus()
