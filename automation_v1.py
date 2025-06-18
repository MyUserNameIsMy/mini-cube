import sys
import time
import threading  # Re-imported threading
import RPi.GPIO as GPIO
from minicubebase import MotorV1, MotorV2

# === Hardware PIN & Device Configuration ===
SERVO_PINS = [12, 16]
HALL_Y = 4
HALL_X = 5
MAGNET_PIN = 6
DEVICE_NAME = '/dev/ttyUSB0'

# === Constants for Movement ===
Z_AXIS_LOWER_DEG = -900
Z_AXIS_RAISE_DEG = 900
Z_AXIS_SPEED = 500

SERVO_LIFT_ANGLE_1 = 95
SERVO_LIFT_ANGLE_2 = 110
SERVO_LOWER_ANGLE = 0

# === GPIO Setup ===
GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PINS[0], GPIO.OUT)
GPIO.setup(SERVO_PINS[1], GPIO.OUT)
GPIO.setup(HALL_Y, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(HALL_X, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(MAGNET_PIN, GPIO.OUT, initial=GPIO.LOW)

# === PWM Initialization ===
pwm1 = GPIO.PWM(SERVO_PINS[0], 50)
pwm2 = GPIO.PWM(SERVO_PINS[1], 50)
pwm1.start(0)
pwm2.start(0)

# === Shared variables for Threaded Servo Control ===
# These variables are shared between the main thread and the servo control threads.
servo1_angle = 0
servo2_angle = 0
servo_running = True
servo_lock = threading.Lock()  # To prevent race conditions when accessing angles


def set_angle_loop(pwm, get_angle_func, lock):
    """
    Thread target function to continuously send PWM signals to a servo.
    This maintains holding torque.
    """
    while servo_running:
        with lock:
            angle = get_angle_func()

        # The formula to convert angle (0-180) to duty cycle (2-12)
        duty_cycle = 2 + (angle / 18.0)
        pwm.ChangeDutyCycle(duty_cycle)
        time.sleep(0.05)  # Refresh rate for the servo signal


# === Motor Initialization ===
motor1_z = MotorV1(DEVICE_NAME, 4)
motor1_x = MotorV1(DEVICE_NAME, 1)
motor2_x = MotorV2(DEVICE_NAME, 1)
motor1_y = MotorV1(DEVICE_NAME, 3)
motor2_y = MotorV2(DEVICE_NAME, 2)

motors = [motor1_z, motor1_x, motor1_y, motor2_x, motor2_y]
for m in motors:
    m.enable_torque()
    time.sleep(0.05)

# Store original modes
MOTOR_V1_ORIGINAL_MODE = 'MULTI_TURN_MODE'
MOTOR_V2_ORIGINAL_MODE = 'EXTENDED_POSITION_MODE'
# Set motor modes
motor1_y.set_mode(MOTOR_V1_ORIGINAL_MODE)
motor2_y.set_mode(MOTOR_V2_ORIGINAL_MODE)
motor1_x.set_mode(MOTOR_V1_ORIGINAL_MODE)
motor2_x.set_mode(MOTOR_V2_ORIGINAL_MODE)
motor1_z.set_mode(MOTOR_V1_ORIGINAL_MODE)


# === Helper Functions for Main Sequence ===

def control_magnet(state):
    """Turns the magnet ON or OFF."""
    if state.upper() == 'ON':
        print("-> Turning magnet ON")
        GPIO.output(MAGNET_PIN, GPIO.HIGH)
    elif state.upper() == 'OFF':
        print("-> Turning magnet OFF")
        GPIO.output(MAGNET_PIN, GPIO.LOW)
    time.sleep(0.5)


def control_z_axis(direction):
    """Lowers or raises the Z-axis motor."""
    move_deg = Z_AXIS_LOWER_DEG if direction.upper() == 'LOWER' else Z_AXIS_RAISE_DEG
    print(f"-> {direction.capitalize()}ing Z-axis by {move_deg} degrees")
    motor1_z.move_deg(move_deg)
    time.sleep(2)


def move_y_one_cell():
    """Moves the robot forward by one 'cell' using the Y-axis motors and HALL_Y sensor."""
    print("-> Moving forward one cell (Y-axis)...")
    motor1_y.set_mode('WHEEL_MODE')
    motor2_y.set_mode('VELOCITY_MODE')
    time.sleep(0.05)

    motor1_y.move_backward()
    motor2_y.move_forward()

    time.sleep(1.5)
    while GPIO.input(HALL_Y) == GPIO.HIGH: time.sleep(0.05)

    motor1_y.stop_move()
    motor2_y.stop_move()
    print("   ...Cell boundary detected.")

    motor1_y.set_mode(MOTOR_V1_ORIGINAL_MODE)
    motor2_y.set_mode(MOTOR_V2_ORIGINAL_MODE)
    time.sleep(0.05)

    motor1_y.move_deg(-200)
    motor2_y.move_deg(200)
    time.sleep(1)
    print("   ...Cell movement finished.")


def move_x_one_cell():
    """Moves the robot sideways by one 'cell' using the X-axis motors and HALL_X sensor."""
    print("-> Moving forward one cell (X-axis)...")
    motor1_x.set_mode('WHEEL_MODE')
    motor2_x.set_mode('VELOCITY_MODE')
    time.sleep(0.05)

    motor1_x.move_forward()
    motor2_x.move_forward()

    time.sleep(1.5)
    while GPIO.input(HALL_X) == GPIO.HIGH: time.sleep(0.05)

    motor1_x.stop_move()
    motor2_x.stop_move()
    print("   ...Cell boundary detected.")

    motor1_x.set_mode(MOTOR_V1_ORIGINAL_MODE)
    motor2_x.set_mode(MOTOR_V2_ORIGINAL_MODE)
    time.sleep(0.05)

    motor1_x.move_deg(-200)
    motor2_x.move_deg(200)
    time.sleep(1)
    print("   ...Cell movement finished.")


def control_servos(state):
    """
    Lifts or lowers the servos by updating the target angles for the background threads.
    """
    global servo1_angle, servo2_angle
    with servo_lock:  # Use the lock to safely modify the shared variables
        if state.upper() == 'LIFT':
            print(f"-> Setting servo target angles to: LIFT ({SERVO_LIFT_ANGLE_1}, {SERVO_LIFT_ANGLE_2})")
            servo1_angle = SERVO_LIFT_ANGLE_1
            servo2_angle = SERVO_LIFT_ANGLE_2
        elif state.upper() == 'LOWER':
            print(f"-> Setting servo target angles to: LOWER ({SERVO_LOWER_ANGLE})")
            servo1_angle = SERVO_LOWER_ANGLE
            servo2_angle = SERVO_LOWER_ANGLE
    # Give the servos time to physically move to the new target position
    time.sleep(1)


# === Main Execution Sequence ===
def main_sequence():
    """Executes the main task sequence."""
    print("\n--- Starting Main Sequence ---")

    control_z_axis('LOWER')
    control_magnet('ON')
    control_z_axis('RAISE')

    move_y_one_cell()
    move_y_one_cell()

    control_servos('LIFT')

    move_x_one_cell()

    control_z_axis('LOWER')
    control_magnet('OFF')
    control_z_axis('RAISE')

    control_servos('LOWER')

    print("\n--- Main Sequence Complete ---")


if __name__ == "__main__":
    # Helper functions to pass to the thread
    def get_servo1_angle():
        return servo1_angle


    def get_servo2_angle():
        return servo2_angle


    # Create and start the servo threads
    servo_thread_1 = threading.Thread(target=set_angle_loop, args=(pwm1, get_servo1_angle, servo_lock))
    servo_thread_2 = threading.Thread(target=set_angle_loop, args=(pwm2, get_servo2_angle, servo_lock))
    servo_thread_1.daemon = True  # Daemon threads exit when the main program exits
    servo_thread_2.daemon = True
    servo_thread_1.start()
    servo_thread_2.start()
    print("Servo control threads started.")

    try:
        input("Press Enter to begin the automated sequence...")
        main_sequence()
    except KeyboardInterrupt:
        print("\nProgram interrupted by user.")
    finally:
        print("\nExiting and cleaning up...")

        # Gracefully stop the servo threads
        print("Stopping servo threads...")
        servo_running = False  # Signal threads to exit their loops
        servo_thread_1.join()  # Wait for threads to finish
        servo_thread_2.join()

        # Stop all hardware
        for m in motors:
            m.stop_move()
            m.disable_torque()
            time.sleep(0.05)
            if m.portHandler.is_open:
                m.portHandler.closePort()

        pwm1.stop()
        pwm2.stop()
        GPIO.cleanup()
        print("Motors, servos, and GPIO cleaned up successfully.")