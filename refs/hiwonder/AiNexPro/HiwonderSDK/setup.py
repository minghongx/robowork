import os
from distutils.core import setup

try:
    os.system('sudo rm -r build')
    os.system('sudo rm -r /usr/local/lib/python3.7/dist-packages/hiwonder')
except:
    pass

setup(name="hiwonder", version="1.0", description="common functions", author="aiden", py_modules=['hiwonder.Misc', 'hiwonder.PID', 'hiwonder.TTS', 'hiwonder.ASR', 'hiwonder.Board', 'hiwonder.Mpu6050', 'hiwonder.Sonar', 'hiwonder.ActionGroupControl', 'hiwonder.PWMServo', 'hiwonder.BusServoCmd'])
