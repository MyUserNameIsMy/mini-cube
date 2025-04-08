import sys
import tty
import termios
import select
import time
from minicubebase import MotorV1, MotorV2

def get_key():
    """Reads key including arrow keys."""
    if select.select([sys.stdin], [], [], 0.0)[0]:
        ch1 = sys.stdin.read(1)
        if ch1 == '\x1b':
            ch2 = sys.stdin.read(1)
            if ch2 == '[':
                ch3 = sys.stdin.read(1)
                return ch1 + ch2 + ch3
        return ch1
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
motor2 = MotorV2(DEVICE_NAME, 1)
motor3 = MotorV1(DEVICE_NAME, 3)
motor4 = MotorV2(DEVICE_NAME, 2)

# Initialize all motors
for m in [motor1, motor2, motor3, motor4]:
    m.enable_torque()

motor1.set_mode('WHEEL_MODE')
motor3.set_mode('WHEEL_MODE')
motor2.set_mode('VELOCITY_MODE')
motor4.set_mode('VELOCITY_MODE')

print("🕹️  Arrow ↑ selects motors 1 & 2 | ↓ selects motors 3 & 4")
print("    Use [w] = forward | [s] = backward | [q] = quit")

# Current selection
selected = [motor1, motor2]

old_settings = set_terminal_raw()
try:
    while True:
        key = get_key()
        if key == '\x1b[A':
            selected = [motor1, motor2]
            print("↑ Selected motors 1 & 2")
        elif key == '\x1b[B':
            selected = [motor3, motor4]
            print("↓ Selected motors 3 & 4")
        elif key == 'w':
            selected[0].move_forward()
            selected[1].move_backward()
        elif key == 's':
            selected[0].move_backward()
            selected[1].move_forward()
        elif key == 'q':
            print("Exiting...")
            break
        elif key is not None:
            for m in selected:
                m.stop_move()
        time.sleep(0.05)

finally:
    restore_terminal_settings(old_settings)
    for m in [motor1, motor2, motor3, motor4]:
        m.stop_move()
        m.disable_torque()
    motor1.portHandler.closePort()
    print("Motors stopped and port closed.")
