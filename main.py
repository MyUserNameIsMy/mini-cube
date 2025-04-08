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

motor1 = MotorV1(DEVICE_NAME, 1)
motor1.enable_torque()
motor1.set_mode('WHEEL_MODE')

try:
    while True:
        ch = getch()
        if ch == 'w':
            motor1.move_forward()
        elif ch == 's':
            motor1.move_backward()
        elif ch == 'q':
            print("Exiting...")
            break
        else:
            motor1.stop_move()
        time.sleep(0.1)
finally:
    motor1.stop_move()
    motor1.disable_torque()
    motor1.portHandler.closePort()

