import sys
import tty
import termios
import select
import time
import RPi.GPIO as GPIO
from minicubebase import MotorV1, MotorV2

# === Configuration ===
# --- Port and Pin Definitions ---
DEVICE_NAME = '/dev/ttyUSB0'
HALL_Y = 5
HALL_X = 4
MAGNET_PIN = 6

# --- Lift Motor Angles (in degrees) ---
# Adjust these values to fine-tune the lift mechanism.
# In MULTI_TURN_MODE, you can use values greater than 360.
LIFT_UP_ANGLE = 2048
LIFT_DOWN_ANGLE = -2048


# === Motor Initialization ===
motor1_z = MotorV1(DEVICE_NAME, 4)
motor1_x = MotorV1(DEVICE_NAME, 1)
motor2_x = MotorV1(DEVICE_NAME, 3)
motor1_y = MotorV2(DEVICE_NAME, 1)
motor2_y = MotorV2(DEVICE_NAME, 2)

# Lifting motors
lift_motor_1 = MotorV1(DEVICE_NAME, 5)
lift_motor_2 = MotorV1(DEVICE_NAME, 7)

motor1_z.set_speed(300)


# === Setup ===
# GPIO setup for the magnet
GPIO.setmode(GPIO.BCM)
GPIO.setup(MAGNET_PIN, GPIO.OUT, initial=GPIO.LOW)

# Consolidate all motors
motors = [motor1_z, motor1_x, motor1_y, motor2_x, motor2_y, lift_motor_1, lift_motor_2]
for m in motors:
    m.enable_torque()


# Store original modes
MOTOR_V1_ORIGINAL_MODE = 'MULTI_TURN_MODE'
MOTOR_V2_ORIGINAL_MODE = 'EXTENDED_POSITION_MODE'

# Set motor operating modes
motor1_x.set_mode(MOTOR_V1_ORIGINAL_MODE)
motor2_x.set_mode(MOTOR_V1_ORIGINAL_MODE)
motor1_y.set_mode(MOTOR_V2_ORIGINAL_MODE)
motor2_y.set_mode(MOTOR_V2_ORIGINAL_MODE)
motor1_z.set_mode(MOTOR_V1_ORIGINAL_MODE)

# Set lifting motors to MULTI_TURN_MODE
lift_motor_1.set_mode(MOTOR_V1_ORIGINAL_MODE)
lift_motor_2.set_mode(MOTOR_V1_ORIGINAL_MODE)


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


def move_y_one_cell(direction, move_further=True):
    """Moves the robot forward by one 'cell' using the Y-axis motors and HALL_Y sensor."""
    print("-> Moving forward one celal (Y-axis)...")
    motor1_y.set_mode('VELOCITY_MODE')
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

    if move_further:
        if direction == 'FORWARD':
            motor1_y.move_backward()
            motor2_y.move_forward()
        elif direction == 'BACKWARD':
            motor1_y.move_forward()
            motor2_y.move_backward()

        while GPIO.input(HALL_Y) == GPIO.LOW: time.sleep(0.05)

        motor1_y.stop_move()
        motor2_y.stop_move()


def move_x_one_cell(direction, move_further=True):
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

    if move_further:
        if direction == 'FORWARD':
            motor1_x.move_backward()
            motor2_x.move_forward()
        elif direction == 'BACKWARD':
            motor1_x.move_forward()
            motor2_x.move_backward()

        while GPIO.input(HALL_Y) == GPIO.LOW: time.sleep(0.05)

        motor1_x.stop_move()
        motor2_x.stop_move()
        time.sleep(1)


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

# === Main Execution Sequence ===
def main_sequence():
    move_y_one_cell('FORWARD')
    move_y_one_cell('FORWARD')
    adjustment(motor1_y, motor2_y, 'FORWARD', 500)
    # control_servos('LIFT')
    # move_x_one_cell('FORWARD')
    # adjustment(motor1_x, motor2_x, 'FORWARD', 500)
    # control_servos('LOWER')
    print("\n--- Main Sequence Complete ---")

if __name__ == "__main__":
    try:
        input("Press Enter to begin the automated sequence...")
        main_sequence()

    except KeyboardInterrupt:
        print("\nProgram interrupted by user.")
    finally:
        print("\nExiting and cleaning up...")

        print("Stopping servo threads...")

        for m in motors:
            m.stop_move()
            m.disable_torque()
            time.sleep(0.05)
            if m.portHandler.is_open:
                m.portHandler.closePort()
        GPIO.cleanup()
        print("Motors, servos, and GPIO cleaned up successfully.")
