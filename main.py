from dynamixel_sdk import *
import os
import sys
import termios
import tty
from minicubebase import *

fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)

def getch():
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

DEVICE_NAME = '/dev/ttyUSB0'

motor1 = MotorV1(1, DEVICE_NAME)
motor1.enable_torque()
motor1.set_mode('WHEEL_MODE')