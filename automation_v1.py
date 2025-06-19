import sys
import time
import threading
import RPi.GPIO as GPIO
from minicubebase import MotorV1, MotorV2

# === Hardware PIN & Device Configuration ===
SERVO_PINS = [12, 16]
HALL_Y = 5
HALL_X = 4
MAGNET_PIN = 6
DEVICE_NAME = '/dev/ttyUSB0'

# === Constants for Movement ===

# --- NEW: Added a home position for the Z-axis ---
Z_AXIS_HOME_POSITION_DEG = 8000

# --- IMPROVED: Specific Z-Axis positions for pickup and drop-off ---
Z_AXIS_LOWER_FOR_PICKUP_DEG = -6000  # Position to lower to when picking up
Z_AXIS_RAISE_AFTER_PICKUP_DEG = 6000  # Lift HIGHER after grabbing the box to ensure clearance
Z_AXIS_LOWER_FOR_DROPOFF_DEG = -6000  # Go LOWER when placing the box to ensure contact
Z_AXIS_RAISE_AFTER_DROPOFF_DEG = Z_AXIS_HOME_POSITION_DEG  # Return to a standard height after dropping off

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
servo1_angle = 0
servo2_angle = 0
servo_running = True
servo_lock = threading.Lock()


def set_angle_loop(pwm, get_angle_func, lock):
    """
    Thread target function to continuously send PWM signals to a servo.
    This maintains holding torque.
    """
    while servo_running:
        with lock:
            angle = get_angle_func()
        duty_cycle = 2 + (angle / 18.0)
        pwm.ChangeDutyCycle(duty_cycle)
        time.sleep(0.05)


# === Motor Initialization ===
motor1_z = MotorV1(DEVICE_NAME, 4)
motor1_x = MotorV1(DEVICE_NAME, 1)
motor2_x = MotorV2(DEVICE_NAME, 1)
motor1_y = MotorV1(DEVICE_NAME, 3)
motor2_y = MotorV2(DEVICE_NAME, 2)

motor1_z.set_speed(300)

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


# --- IMPROVED: This function now moves to a specific target degree ---
def control_z_axis(target_position_deg):
    """Moves the Z-axis motor to a specific degree position."""
    print(f"-> Moving Z-axis to position: {target_position_deg} degrees")
    motor1_z.move_deg(target_position_deg)
    # The sleep time might need adjustment depending on the travel distance
    time.sleep(2.5)


def move_y_one_cell(direction):
    """Moves the robot forward by one 'cell' using the Y-axis motors and HALL_Y sensor."""
    print("-> Moving forward one cell (Y-axis)...")
    motor1_y.set_mode('WHEEL_MODE')
    motor2_y.set_mode('VELOCITY_MODE')
    time.sleep(0.05)

    if direction == 'FORWARD':
        motor1_y.move_backward()
        motor2_y.move_forward()
    elif direction == 'BACKWARD':
        motor1_y.move_forward()
        motor2_y.move_backward()

    time.sleep(1.5)
    while GPIO.input(HALL_Y) == GPIO.HIGH: time.sleep(0.05)

    motor1_y.stop_move()
    motor2_y.stop_move()


def move_x_one_cell(direction):
    """Moves the robot sideways by one 'cell' using the X-axis motors and HALL_X sensor."""
    print("-> Moving forward one cell (X-axis)...")
    motor1_x.set_mode('WHEEL_MODE')
    motor2_x.set_mode('VELOCITY_MODE')
    time.sleep(0.05)

    if direction == 'FORWARD':
        motor1_x.move_backward()
        motor2_x.move_forward()
    elif direction == 'BACKWARD':
        motor1_x.move_forward()
        motor2_x.move_backward()

    time.sleep(1.5)
    while GPIO.input(HALL_X) == GPIO.HIGH: time.sleep(0.05)

    motor1_x.stop_move()
    motor2_x.stop_move()


def adjustment(motor1, motor2, direction, deg):
    motor1.set_mode(MOTOR_V1_ORIGINAL_MODE)
    motor2.set_mode(MOTOR_V2_ORIGINAL_MODE)
    time.sleep(0.05)
    if direction == 'FORWARD':
        motor1.move_deg(-deg)
        motor2.move_deg(deg)
    elif direction == 'BACKWARD':
        motor1.move_deg(deg)
        motor2.move_deg(-deg)
    time.sleep(1)
    motor1.stop_move()
    motor2.stop_move()

def control_servos(state):
    """Lifts or lowers the servos by updating the target angles for the background threads."""
    global servo1_angle, servo2_angle
    with servo_lock:
        if state.upper() == 'LIFT':
            print(f"-> Setting servo target angles to: LIFT ({SERVO_LIFT_ANGLE_1}, {SERVO_LIFT_ANGLE_2})")
            servo1_angle = SERVO_LIFT_ANGLE_1
            servo2_angle = SERVO_LIFT_ANGLE_2
        elif state.upper() == 'LOWER':
            print(f"-> Setting servo target angles to: LOWER ({SERVO_LOWER_ANGLE})")
            servo1_angle = SERVO_LOWER_ANGLE
            servo2_angle = SERVO_LOWER_ANGLE
    time.sleep(1)


# === Main Execution Sequence ===
def main_sequence():
    """Executes the main task sequence with improved height control."""
    print("\n--- Starting Main Sequence ---")

    # --- Step 1: Pick up the box ---
    print("\n[PHASE 1: PICKUP]")
    control_z_axis(Z_AXIS_LOWER_FOR_PICKUP_DEG)
    control_magnet('ON')
    motor1_z.set_deg(Z_AXIS_HOME_POSITION_DEG)  # Use the higher lift value
    time.sleep(3)
    # --- Step 2: Move to the drop-off location ---
    print("\n[PHASE 2: TRAVEL]")
    move_y_one_cell('FORWARD')
    move_y_one_cell('FORWARD')
    adjustment(motor1_y, motor2_y, 'FORWARD', 700)
    control_servos('LIFT')

    # move_x_one_cell()
    # move_x_one_cell()
    #
    # # --- Step 3: Drop off the box ---
    # print("\n[PHASE 3: DROPOFF]")
    # control_z_axis(Z_AXIS_LOWER_FOR_DROPOFF_DEG) # Use the lower placement value
    # control_magnet('OFF')
    # motor1_z.set_deg(Z_AXIS_HOME_POSITION_DEG) # Return to normal height
    #
    # # --- Step 4: Reset servo positions ---
    # print("\n[PHASE 4: RESET]")
    # control_servos('LOWER')

    print("\n--- Main Sequence Complete ---")


if __name__ == "__main__":
    def get_servo1_angle():
        return servo1_angle


    def get_servo2_angle():
        return servo2_angle


    servo_thread_1 = threading.Thread(target=set_angle_loop, args=(pwm1, get_servo1_angle, servo_lock))
    servo_thread_2 = threading.Thread(target=set_angle_loop, args=(pwm2, get_servo2_angle, servo_lock))
    servo_thread_1.daemon = True
    servo_thread_2.daemon = True
    servo_thread_1.start()
    servo_thread_2.start()
    print("Servo control threads started.")

    try:
        # --- NEW: Initialize Z-axis to a known starting position ---
        print(f"\nInitializing Z-axis to HOME position ({Z_AXIS_HOME_POSITION_DEG} degrees)...")
        motor1_z.set_deg(Z_AXIS_HOME_POSITION_DEG)
        time.sleep(2.5)  # Wait for the motor to reach its position
        print("Initialization complete. Ready to start.")

        input("Press Enter to begin the automated sequence...")
        main_sequence()
    except KeyboardInterrupt:
        print("\nProgram interrupted by user.")
    finally:
        print("\nExiting and cleaning up...")

        print("Stopping servo threads...")
        servo_running = False
        servo_thread_1.join()
        servo_thread_2.join()

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
