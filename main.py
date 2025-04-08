import sys
import tty
import termios
import select
import time
from minicubebase import *  # This should contain your MotorV1 class

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

# === Motor Setup ===
DEVICE_NAME = '/dev/ttyUSB0'
motor1 = MotorV1(DEVICE_NAME, 1)
motor2 = MotorV1(DEVICE_NAME, 3)

# Initialize both motors
for motor in [motor1, motor2]:
    motor.enable_torque()
    motor.set_mode('WHEEL_MODE')

print("🕹️  Use [w] to move forward, [s] to move backward, [q] to quit.")

# === Input Loop ===
old_settings = set_terminal_raw()
try:
    while True:
        key = get_key()
        if key == 'w':
            motor1.move_forward()
            motor2.move_forward()
        elif key == 's':
            motor1.move_backward()
            motor2.move_backward()
        elif key == 'q':
            print("Exiting...")
            break
        elif key is not None:
            motor1.stop_move()
            motor2.stop_move()

        time.sleep(0.05)

finally:
    restore_terminal_settings(old_settings)
    motor1.stop_move()
    motor2.stop_move()
    motor1.disable_torque()
    motor2.disable_torque()
    motor1.portHandler.closePort()
    print("Motors stopped and port closed.")
