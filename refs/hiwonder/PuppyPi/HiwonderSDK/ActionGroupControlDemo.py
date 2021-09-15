import time
import threading
import ActionGroupControl as AGC

print('''
**********************************************************
*********功能:幻尔科技SpiderPi扩展板，动作组控制例程********
**********************************************************
----------------------------------------------------------
Official website:http://www.hiwonder.com
Online mall:https://huaner.tmall.com/
----------------------------------------------------------
以下指令均需在LX终端使用，LX终端可通过ctrl+alt+t打开，或点
击上栏的黑色LX终端图标。
----------------------------------------------------------
Usage:
    sudo python3 ActionGroupControlDemo.py
----------------------------------------------------------
Version: --V1.2  2020/12/23
----------------------------------------------------------
Tips:
 * 按下Ctrl+C可关闭此次程序运行，若失败请多次尝试！
----------------------------------------------------------
''')

# 动作组需要保存在路径/home/pi/SpiderPi/ActionGroups下
AGC.runActionGroup('stand_low')  # 参数为动作组的名称，不包含后缀，以字符形式传入
AGC.runActionGroup('go_forward_low', times=2)  # 第二个参数为运行动作次数，默认1, 当为0时表示循环运行

time.sleep(1)

th = threading.Thread(target=AGC.runActionGroup, args=('turn_right_low', 0), daemon=True)  # 运行动作函数是阻塞式的，如果要循环运行一段时间后停止，请用线程来开启
th.start()
time.sleep(3)
AGC.stopAction()  # 3秒后发出停止指令
while th.is_alive(): # 等待动作完全停止
    time.sleep(0.01)
AGC.runActionGroup('stand_low')
