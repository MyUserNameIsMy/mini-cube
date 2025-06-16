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

motor1_z = MotorV1(DEVICE_NAME, 4)
motor1_x = MotorV1(DEVICE_NAME, 1)
motor2_x = MotorV2(DEVICE_NAME, 1)
motor1_y = MotorV1(DEVICE_NAME, 3)
motor2_y = MotorV2(DEVICE_NAME, 2)


motors = [motor1_z, motor1_x, motor1_y, motor2_x, motor2_y]
for m in motors:
    m.enable_torque()

motor1_x.set_mode('MULTI_TURN_MODE')
motor1_y.set_mode('MULTI_TURN_MODE')
motor2_x.set_mode('EXTENDED_POSITION_MODE')
motor2_y.set_mode('EXTENDED_POSITION_MODE')

motor1_z.set_mode('MULTI_TURN_MODE')

print("🕹️  Arrow ↑ selects motors 1 & 2 | ↓ selects motors 3 & 4")
print("    Use [w] = forward | [s] = backward | [q] = quit")
print("    Use ↑ for servos 80° & 78°, ↓ for servos 0°")

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

# === Motor selection ===
selected = [motor1_y, motor2_y]
try:
    while True:
        key = get_key()

        if key == 'w':
            selected[0].move_deg(-8000)
            selected[1].move_deg(8000)
        elif key == 'q':
            print("Exiting...")
            break
        elif key is not None:
            motor1_z.stop_move()
            for m in selected:
                m.stop_move()
        time.sleep(0.05)

finally:
    restore_terminal_settings(old_settings)
    servo_running = False
    servo_thread_1.join()
    servo_thread_2.join()
    for m in motors:
        m.stop_move()
        m.disable_torque()
        m.portHandler.closePort()
    pwm1.stop()
    pwm2.stop()
    GPIO.cleanup()
    print("Motors and servos stopped, GPIO cleaned up.")
