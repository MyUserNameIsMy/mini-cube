import sys
import termios
import tty
import time
import select
from minicubebase import *  # your MotorV1 class

def get_key():
    if select.select([sys.stdin], [], [], 0.0)[0]:
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

print("Hold [w] for forward, [s] for backward, [q] to quit.")

old_settings = set_terminal_raw()

try:
    while True:
        key = get_key()
        if key == 'w':
            motor1.move_forward()
        elif key == 's':
            motor1.move_backward()
        elif key == 'q':
            print("Exiting...")
            break
        else:
            motor1.stop_move()

        time.sleep(0.05)

finally:
    restore_terminal_settings(old_settings)
    motor1.stop_move()
    motor1.disable_torque()
    motor1.portHandler.closePort()
