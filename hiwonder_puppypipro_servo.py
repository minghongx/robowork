import pigpio
import serial
import RPi.GPIO as GPIO


class PWMServos:

    def __init__(self, pwm_servo_id: int):
        """
        扩展板上 1 --> ID 12
        扩展板上 2 --> ID 13
        Note: ID 12 为单目摄像头舵机
        """
        self.__id = pwm_servo_id

    def set_pwm_servo_pulse(self, pulse: int):
        """
        驱动 PWM servo 转到指定角度
        :param pulse: PWM 脉宽  note: 舵机的转动范围 0-180deg，对应的脉宽为 500-2500us
        """

        # Boundary limits validation
        pulse = pulse if pulse in range(500, 2500) else \
                500 if pulse < 0 else \
                2500

        pigpio.pi().set_servo_pulsewidth(self.id, pulse)

    @property
    def id(self):
        return self.__id


# FIXME: Serial bus servo ID 0-235 用户可设置. 舵机 ID 是否输入正确每个函数都没有校验

def stop_serial_bus_servo(serial_bus_servo_id: int):  # FIXME: 有没有办法能一次获取所有串行接口舵机的 id
    _UART.write_cmd_to_serial_bus_servo(serial_bus_servo_id, _UART.MOVE_STOP)


def unload_serial_bus_servo(serial_bus_servo_id: int):  # 掉电
    _UART.write_cmd_to_serial_bus_servo(serial_bus_servo_id, _UART.LOAD_OR_UNLOAD_WRITE, 0)  # 0 represents UNLOAD


def set_serial_bus_servo_pulse_and_rotating_duration(serial_bus_servo_id: int, pulse: int, rotating_duration: int):
    """
    驱动串行总线舵机转到指定角度
    :param serial_bus_servo_id: 要驱动的串行总线舵机 id
    :param pulse: 角度  note: 舵机的转动范围 0-240deg，对应的脉宽为 0-1000
    :param rotating_duration 转动时长 (ms)
    """

    # Boundary limits validation
    pulse = pulse if pulse in range(0, 1000) else \
            0 if pulse < 0 else \
            1000
    rotating_duration = rotating_duration if rotating_duration in range(0, 3000) else \
                        0 if rotating_duration < 0 else \
                        3000

    _UART.write_cmd_to_serial_bus_servo(serial_bus_servo_id, _UART.MOVE_TIME_WRITE, pulse, rotating_duration)


class _UART:
    """幻尔科技总线舵机通信"""

    # 初始化串口. 通信波特率为 115200
    _SERIAL_HANDLE = serial.Serial("/dev/ttyAMA0", 115200)

    _FRAME_HEADER = 0x55

    # UART commands
    MOVE_TIME_WRITE = 1
    MOVE_TIME_READ = 2
    MOVE_TIME_WAIT_WRITE = 7
    MOVE_TIME_WAIT_READ = 8
    MOVE_START = 11
    MOVE_STOP = 12
    ID_WRITE = 13
    ID_READ = 14
    ANGLE_OFFSET_ADJUST = 17
    ANGLE_OFFSET_WRITE = 18
    ANGLE_OFFSET_READ = 19
    ANGLE_LIMIT_WRITE = 20
    ANGLE_LIMIT_READ = 21
    VIN_LIMIT_WRITE = 22
    VIN_LIMIT_READ = 23
    TEMP_MAX_LIMIT_WRITE = 24
    TEMP_MAX_LIMIT_READ = 25
    TEMP_READ = 26
    VIN_READ = 27
    POS_READ = 28
    OR_MOTOR_MODE_WRITE = 29
    OR_MOTOR_MODE_READ = 30
    LOAD_OR_UNLOAD_WRITE = 31
    LOAD_OR_UNLOAD_READ = 32
    LED_CTRL_WRITE = 33
    LED_CTRL_READ = 34
    LED_ERROR_WRITE = 35
    LED_ERROR_READ = 36

    @classmethod
    def write_cmd_to_serial_bus_servo(cls, serial_bus_servo_id=None, cmd=None, dat1=None, dat2=None):

        # 配置单线串口为输出
        rx_pin = 7
        tx_pin = 13
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        GPIO.setup(rx_pin, GPIO.OUT)  # 配置RX_CON 即 GPIO17 为输出
        GPIO.output(rx_pin, 0)
        GPIO.setup(tx_pin, GPIO.OUT)  # 配置TX_CON 即 GPIO27 为输出
        GPIO.output(tx_pin, 1)

        buf = bytearray(b'\x55\x55')  # 帧头
        buf.append(serial_bus_servo_id)

        # 指令长度
        if dat1 is None and dat2 is None:
            buf.append(3)
        elif dat1 is not None and dat2 is None:
            buf.append(4)
        elif dat1 is not None and dat2 is not None:
            buf.append(7)

        buf.append(cmd)  # 指令

        # 写数据
        if dat1 is None and dat2 is None:
            pass
        elif dat1 is not None and dat2 is None:
            buf.append(dat1 & 0xff)  # 偏差
        elif dat1 is not None and dat2 is not None:
            buf.extend([(0xff & dat1), (0xff & (dat1 >> 8))])  # 分低8位 高8位 放入缓存
            buf.extend([(0xff & dat2), (0xff & (dat2 >> 8))])  # 分低8位 高8位 放入缓存

        # 校验和
        buf.append(cls.__check_sum(buf))

        # 发送
        cls._SERIAL_HANDLE.write(buf)

    @classmethod
    def __check_sum(cls, buf):
        # 计算校验和
        sum = 0x00
        for b in buf:  # 求和
            sum += b
        sum = sum - 0x55 - 0x55  # 去掉命令开头的两个 0x55
        sum = ~sum  # 取反
        return sum & 0xff


'''
Reference
https://en.wikipedia.org/wiki/Servo_(radio_control)
https://bbs.hiwonder.com/postDetails/1005
https://bbs.hiwonder.com/postDetails/937
https://bbs.hiwonder.com/postDetails/588
https://bbs.hiwonder.com/postDetails/941
https://wiki.fashionrobo.com/uartbasic/uart-servo-introduction/
https://zhuanlan.zhihu.com/p/150504364
https://www.zhihu.com/question/54184654
'''
