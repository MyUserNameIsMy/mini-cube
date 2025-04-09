import sys
import tty
import termios
import select
import time
import threading
import RPi.GPIO as GPIO
from minicubebase import MotorV1, MotorV2

# === Servo Setup ===
SERVO_PINS = [12, 16]

GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PINS[0], GPIO.OUT)
GPIO.setup(SERVO_PINS[1], GPIO.OUT)

pwm1 = GPIO.PWM(SERVO_PINS[0], 50)
pwm2 = GPIO.PWM(SERVO_PINS[1], 50)
pwm1.start(0)
pwm2.start(0)

# === Shared angles and control flags ===
servo1_angle = 0
servo2_angle = 0
servo_running = True
servo_lock = threading.Lock()

# === Threaded angle update loops ===
def set_angle_loop(pwm, get_angle_func):
    while servo_running:
        with servo_lock:
            angle = get_angle_func()
        duty_cycle = 2 + (angle / 18)
        pwm.ChangeDutyCycle(duty_cycle)
        time.sleep(0.1)

# === Motor Setup ===
DEVICE_NAME = '/dev/ttyUSB0'

motor1 = MotorV1(DEVICE_NAME, 1)
motor2 = MotorV2(DEVICE_NAME, 1)
motor3 = MotorV1(DEVICE_NAME, 3)
motor4 = MotorV2(DEVICE_NAME, 2)

for m in [motor1, motor2, motor3, motor4]:
    m.enable_torque()

motor1.set_mode('WHEEL_MODE')
motor3.set_mode('WHEEL_MODE')
motor2.set_mode('VELOCITY_MODE')
motor4.set_mode('VELOCITY_MODE')

print("🕹️  Arrow ↑ selects motors 1 & 2 | ↓ selects motors 3 & 4")
print("    Use [w] = forward | [s] = backward | [q] = quit")
print("    Use ↑ for servos 80° & 78°, ↓ for servos 0°")

# === Motor selection ===
selected = [motor1, motor2]

# === Terminal key handling ===
def get_key():
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

# === Start PWM threads ===
def get_servo1_angle(): return servo1_angle
def get_servo2_angle(): return servo2_angle

servo_thread_1 = threading.Thread(target=set_angle_loop, args=(pwm1, get_servo1_angle))
servo_thread_2 = threading.Thread(target=set_angle_loop, args=(pwm2, get_servo2_angle))
servo_thread_1.daemon = True
servo_thread_2.daemon = True
servo_thread_1.start()
servo_thread_2.start()

old_settings = set_terminal_raw()
try:
    while True:
        key = get_key()

        if key == '\x1b[A':  # Up Arrow - Select motors 1 & 2
            selected = [motor1, motor2]
            print("↑ Selected motors 1 & 2")
            with servo_lock:
                servo1_angle = 80
                servo2_angle = 78
            print("Setting servos: pwm1 → 80°, pwm2 → 78°")
        elif key == '\x1b[B':  # Down Arrow - Select motors 3 & 4
            selected = [motor3, motor4]
            print("↓ Selected motors 3 & 4")
            with servo_lock:
                servo1_angle = 0
                servo2_angle = 0
            print("Setting servos: pwm1 → 0°, pwm2 → 0°")
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
    servo_running = False
    servo_thread_1.join()
    servo_thread_2.join()
    for m in [motor1, motor2, motor3, motor4]:
        m.stop_move()
        m.disable_torque()
    motor1.portHandler.closePort()
    pwm1.stop()
    pwm2.stop()
    GPIO.cleanup()
    print("Motors and servos stopped, GPIO cleaned up.")
