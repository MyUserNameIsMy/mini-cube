import sys
import tty
import termios
import select
import time
import threading
import RPi.GPIO as GPIO
from minicubebase import MotorV1, MotorV2

# === Hardware PIN Configuration ===
SERVO_PINS = [12, 16]

HALL_Y = 4
HALL_X = 5

DEVICE_NAME = '/dev/ttyUSB0'

# === GPIO Setup ===
GPIO.setmode(GPIO.BCM)
# Servos
GPIO.setup(SERVO_PINS[0], GPIO.OUT)
GPIO.setup(SERVO_PINS[1], GPIO.OUT)
# Hall Effect Sensor
GPIO.setup(HALL_Y, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(HALL_X, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# === PWM & Servo Initialization ===
pwm1 = GPIO.PWM(SERVO_PINS[0], 50)
pwm2 = GPIO.PWM(SERVO_PINS[1], 50)
pwm1.start(0)
pwm2.start(0)

# Shared variables for servo control
servo1_angle = 0
servo2_angle = 0
servo_running = True
servo_lock = threading.Lock()


def set_angle_loop(pwm, get_angle_func):
    """Thread target to continuously update a servo's angle."""
    while servo_running:
        with servo_lock:
            angle = get_angle_func()
        duty_cycle = 2 + (angle / 18)
        pwm.ChangeDutyCycle(duty_cycle)
        time.sleep(0.1)


# === Motor Initialization ===
motor1_z = MotorV1(DEVICE_NAME, 4)
motor1_x = MotorV1(DEVICE_NAME, 1)
motor2_x = MotorV2(DEVICE_NAME, 1)
motor1_y = MotorV1(DEVICE_NAME, 3)
motor2_y = MotorV2(DEVICE_NAME, 2)

motors = [motor1_z, motor1_x, motor1_y, motor2_x, motor2_y]
for m in motors:
    m.enable_torque()

# Store original modes to revert back to them
MOTOR_V1_ORIGINAL_MODE = 'MULTI_TURN_MODE'
MOTOR_V2_ORIGINAL_MODE = 'EXTENDED_POSITION_MODE'

motor1_y.set_mode(MOTOR_V1_ORIGINAL_MODE)
motor2_y.set_mode(MOTOR_V2_ORIGINAL_MODE)
motor1_x.set_mode(MOTOR_V1_ORIGINAL_MODE)
motor2_x.set_mode(MOTOR_V2_ORIGINAL_MODE)

print("Motor and Sensor Control Script Initialized.")
print("Forward motion is timed. Backward motion uses velocity mode until magnet is gone.")
print("Press 'q' to quit.")


# === Terminal Input Handling ===
def get_key():
    """Check for and return a key press without blocking."""
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


# === Main Application ===
old_settings = set_terminal_raw()


# Start servo control threads
def get_servo1_angle(): return servo1_angle


def get_servo2_angle(): return servo2_angle


servo_thread_1 = threading.Thread(target=set_angle_loop, args=(pwm1, get_servo1_angle))
servo_thread_2 = threading.Thread(target=set_angle_loop, args=(pwm2, get_servo2_angle))
servo_thread_1.daemon = True
servo_thread_2.daemon = True
servo_thread_1.start()
servo_thread_2.start()

running = True
forward = True

try:
    while running:
        key = get_key()
        if key == 'q':
            running = False
            break

        if forward:
            print('Moving till magnet found')
            motor1_y.set_mode('WHEEL_MODE')
            motor2_y.set_mode('VELOCITY_MODE')

            motor1_y.move_backward()
            motor2_y.move_forward()

            time.sleep(0.05)
            while GPIO.input(HALL_Y) == GPIO.HIGH:
                if get_key() == 'q':
                    running = False
                    break
                time.sleep(0.05)
            if not running: break

            motor1_y.stop_move()
            motor2_y.stop_move()
            print("Magnet detected. Motors stopped.")

            time.sleep(0.05)
            print("Adjusting itself started.")
            motor1_y.set_mode(MOTOR_V1_ORIGINAL_MODE)
            motor2_y.set_mode(MOTOR_V2_ORIGINAL_MODE)

            motor1_y.move_deg(-600)
            motor2_y.move_deg(600)
            time.sleep(1)
            motor1_y.stop_move()
            motor2_y.stop_move()
            print("Adjusting itself ended.")
            forward = not forward
        else:

            print("Preparing for backward motion...")

            motor1_y.set_mode('WHEEL_MODE')
            motor2_y.set_mode('VELOCITY_MODE')

            print("Moving backward until magnet is gone...")
            motor1_y.move_forward()
            motor2_y.move_backward()

            time.sleep(0.05)
            while GPIO.input(HALL_Y) == GPIO.HIGH:

                if get_key() == 'q':
                    running = False
                    break
                time.sleep(0.05)
            if not running: break

            motor1_y.stop_move()
            motor2_y.stop_move()
            print("Magnet detected. Motors stopped.")

            time.sleep(0.05)
            print("Adjusting itself started.")
            motor1_y.set_mode(MOTOR_V1_ORIGINAL_MODE)
            motor2_y.set_mode(MOTOR_V2_ORIGINAL_MODE)

            motor1_y.move_deg(1000)
            motor2_y.move_deg(-1000)
            time.sleep(1)
            motor1_y.stop_move()
            motor2_y.stop_move()
            print("Adjusting itself ended.")

            forward = not forward

    # Common servo sequence after any move
    if running:
        print("Executing servo sequence...")
        with servo_lock:
            servo1_angle = 95
            servo2_angle = 110
        time.sleep(2)
        with servo_lock:
            servo1_angle = 0
            servo2_angle = 0
        time.sleep(2)

finally:
    print("\nExiting program...")
restore_terminal_settings(old_settings)

# Signal threads to stop
servo_running = False
servo_thread_1.join()
servo_thread_2.join()

# Stop all hardware
for m in motors:
    m.stop_move()
    m.disable_torque()
    if m.portHandler.is_open:
        m.portHandler.closePort()

pwm1.stop()
pwm2.stop()
GPIO.cleanup()

print("Motors, servos, and GPIO cleaned up successfully.")
