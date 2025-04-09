import sys
import tty
import termios
import select
import time
import RPi.GPIO as GPIO
from minicubebase import MotorV1, MotorV2

# === Servo Setup ===
SERVO_PINS = [12, 16]

# Setup GPIO for servos
GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PINS[0], GPIO.OUT)
GPIO.setup(SERVO_PINS[1], GPIO.OUT)

# Set up PWM for servos at 50Hz
pwm1 = GPIO.PWM(SERVO_PINS[0], 50)
pwm2 = GPIO.PWM(SERVO_PINS[1], 50)
pwm1.start(0)
pwm2.start(0)

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
print("    Use ↑ for servos 100°, ↓ for servos 0°")

# Current motor selection
selected = [motor1, motor2]

# Key Input Functions
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

# Servo Angle Function
def set_angle(pwm, angle):
    duty_cycle = 2 + (angle / 18)  # Convert angle to duty cycle
    pwm.ChangeDutyCycle(duty_cycle)
    time.sleep(0.5)
    pwm.ChangeDutyCycle(0)

old_settings = set_terminal_raw()
try:
    while True:
        key = get_key()

        if key == '\x1b[A':  # Up Arrow - Select motors 1 & 2
            selected = [motor1, motor2]
            print("↑ Selected motors 1 & 2")
            time.sleep(0.1)  # Short delay for stability
            set_angle(pwm1, 100)
            # set_angle(pwm2, 100)
            # print("Setting servos to 100°")
        elif key == '\x1b[B':  # Down Arrow - Select motors 3 & 4
            selected = [motor3, motor4]
            print("↓ Selected motors 3 & 4")
            time.sleep(0.1)  # Short delay for stability
            set_angle(pwm1, 0)
            # set_angle(pwm2, 0)
            # print("Setting servos to 0°")
        elif key == 'w':  # Move motors forward
            selected[0].move_forward()
            selected[1].move_backward()
        elif key == 's':  # Move motors backward
            selected[0].move_backward()
            selected[1].move_forward()
        elif key == 'q':  # Exit program
            print("Exiting...")
            break
        # Stop motors when no key is pressed
        elif key is not None:
            for m in selected:
                m.stop_move()
        time.sleep(0.05)

finally:
    restore_terminal_settings(old_settings)
    for m in [motor1, motor2, motor3, motor4]:
        m.stop_move()
        m.disable_torque()
    motor1.portHandler.closePort()  # Both motors use the same port
    pwm1.stop()
    pwm2.stop()
    GPIO.cleanup()
    print("Motors and servos stopped, GPIO cleaned up.")
