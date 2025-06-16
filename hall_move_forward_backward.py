import sys
import tty
import termios
import select
import time
import threading
import RPi.GPIO as GPIO
from minicubebase import MotorV1, MotorV2

# === Hardware Pin Configuration ===
SERVO_PINS = [12, 16]
HALL_SENSOR_PIN = 4
DEVICE_NAME = '/dev/ttyUSB0'  # Serial port for motors

# === GPIO Setup ===
GPIO.setmode(GPIO.BCM)
# Servos
GPIO.setup(SERVO_PINS[0], GPIO.OUT)
GPIO.setup(SERVO_PINS[1], GPIO.OUT)
# Hall Effect Sensor
GPIO.setup(HALL_SENSOR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

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
MOTOR1_Y_ORIGINAL_MODE = 'MULTI_TURN_MODE'
MOTOR2_Y_ORIGINAL_MODE = 'EXTENDED_POSITION_MODE'
motor1_y.set_mode(MOTOR1_Y_ORIGINAL_MODE)
motor2_y.set_mode(MOTOR2_Y_ORIGINAL_MODE)

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
            print("Moving forward for a fixed duration...")
            motor1_y.move_deg(-11800)
            motor2_y.move_deg(11800)
            time.sleep(8)
            forward = not forward
        else:  # Backward motion with sensor
            print("Waiting for the area to be clear of any magnets...")
            while GPIO.input(HALL_SENSOR_PIN) == GPIO.LOW:
                if get_key() == 'q':
                    running = False
                    break
                time.sleep(0.1)
            if not running: break

            print("Preparing for backward motion...")
            # Disable torque, change mode, re-enable torque
            motor1_y.set_mode('WHEEL_MODE')
            motor2_y.set_mode('VELOCITY_MODE')

            print("Moving backward until magnet is gone...")
            motor1_y.move_forward()
            motor2_y.move_backward()

            # Loop while magnet is detected (sensor pin is LOW)
            while GPIO.input(HALL_SENSOR_PIN) == GPIO.HIGH:
                print(f'DEBUG HALL_SENSOR_PIN: {GPIO.input(HALL_SENSOR_PIN)}')
                if get_key() == 'q':
                    running = False
                    break
                time.sleep(0.05)
            print(f'DEBUG HALL_SENSOR_PIN: {GPIO.input(HALL_SENSOR_PIN)}')
            if not running: break

            # Stop motors
            motor1_y.stop_move()
            motor2_y.stop_move()
            print("Magnet not detected. Motors stopped.")

            # Revert motors to original state
            print("Reverting motor modes...")
            motor1_y.set_mode(MOTOR1_Y_ORIGINAL_MODE)
            motor2_y.set_mode(MOTOR2_Y_ORIGINAL_MODE)

            forward = not forward

        # Common servo sequence after any move
        if running:
            print("Executing servo sequence...")
            with servo_lock:
                servo1_angle = 95
                servo2_angle = 110
            time.sleep(1)
            with servo_lock:
                servo1_angle = 0
                servo2_angle = 0
            time.sleep(1)

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
