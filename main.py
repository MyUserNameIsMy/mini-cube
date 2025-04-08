from dynamixel_sdk import *
import os
import sys
import termios
import tty
import time
import select
from minicubebase import *

def get_pressed_key(timeout=0.01):
    dr, dw, de = select.select([sys.stdin], [], [], timeout)
    if dr:
        return sys.stdin.read(1)
    return None

def set_terminal_raw():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    return old_settings

def restore_terminal_settings(old_settings):
    termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_settings)

DEVICE_NAME = '/dev/ttyUSB0'
motor1 = MotorV1(DEVICE_NAME, 1)
motor1.enable_torque()
motor1.set_mode('WHEEL_MODE')

print("Hold [w] to move forward, [s] to move backward, [q] to quit.")

old_settings = set_terminal_raw()

try:
    while True:
        key = get_pressed_key()
        print(key)
        if key == 'w':
            motor1.move_forward()
        elif key == 's':
            motor1.move_backward()
        elif key == 'q':
            print("Quitting...")
            break
        elif key is not None:
            pass
            # motor1.stop_move()
        else:
            pass
            # motor1.stop_move()
        time.sleep(0.01)

finally:
    restore_terminal_settings(old_settings)
    motor1.stop_move()
    motor1.disable_torque()
    motor1.portHandler.closePort()
