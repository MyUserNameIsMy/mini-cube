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


# === Setup ===
# GPIO setup for the magnet
GPIO.setmode(GPIO.BCM)
GPIO.setup(MAGNET_PIN, GPIO.OUT, initial=GPIO.LOW)

# Consolidate all motors
motors = [motor1_z, motor1_x, motor1_y, motor2_x, motor2_y, lift_motor_1, lift_motor_2]
for m in motors:
    m.enable_torque()

# Set motor operating modes
motor1_x.set_mode('WHEEL_MODE')
motor2_x.set_mode('WHEEL_MODE')
motor1_y.set_mode('VELOCITY_MODE')
motor2_y.set_mode('VELOCITY_MODE')
motor1_z.set_mode('WHEEL_MODE')

# Set lifting motors to MULTI_TURN_MODE
lift_motor_1.set_mode('MULTI_TURN_MODE')
lift_motor_2.set_mode('MULTI_TURN_MODE')


# --- Print Controls ---
print("Motor Control Program")
print("  [1] / [2] - Select motor pairs (X/Y)")
print("  [w] / [s] - Move selected pair")
print(f"  [↑] - Lift Up (to {LIFT_UP_ANGLE}°)")
print(f"  [↓] - Lift Down (to {LIFT_DOWN_ANGLE}°)")
print("  [u] / [d] - Z-axis motor control")
print("  [t] / [o] - Magnet ON/OFF")
print("  [q] - Quit")


# === Terminal Input Functions ===
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


# === Main Program Loop ===
if __name__ == "__main__":
    old_settings = set_terminal_raw()
    selected = [motor1_x, motor2_x]

    try:
        while True:
            key = get_key()

            if key == '\x1b[A':  # Up Arrow
                print(f"Lifting UP to {LIFT_UP_ANGLE} degrees")
                lift_motor_1.move_deg(LIFT_UP_ANGLE)
                lift_motor_2.move_deg(LIFT_UP_ANGLE)
            elif key == '\x1b[B':  # Down Arrow
                print(f"Lifting DOWN to {LIFT_DOWN_ANGLE} degrees")
                lift_motor_1.move_deg(LIFT_DOWN_ANGLE)
                lift_motor_2.move_deg(LIFT_DOWN_ANGLE)
            elif key == '1':
                selected = [motor1_x, motor2_x]
                print("Selected motors 1 & 3 (X-axis)")
            elif key == '2':
                selected = [motor1_y, motor2_y]
                print("Selected motors 1 & 2 (Y-axis)")
            elif key == 'u':
                motor1_z.move_forward()
            elif key == 'd':
                motor1_z.move_backward()
            elif key == 'w':
                selected[0].move_forward()
                selected[1].move_backward()
            elif key == 's':
                selected[0].move_backward()
                selected[1].move_forward()
            elif key == 't':
                GPIO.output(MAGNET_PIN, GPIO.HIGH)
                print("MAGNET -> ON")
            elif key == 'o':
                GPIO.output(MAGNET_PIN, GPIO.LOW)
                print("MAGNET -> OFF")
            elif key == 'q':
                print("Exiting...")
                break
            elif key is not None:
                motor1_x.stop_move()
                motor2_x.stop_move()
                motor1_z.stop_move()
                motor1_y.stop_move()
                motor2_y.stop_move()

            time.sleep(0.05)

    finally:
        restore_terminal_settings(old_settings)
        print(f"Moving lift to safe position ({LIFT_DOWN_ANGLE}°)...")
        lift_motor_1.move_deg(LIFT_DOWN_ANGLE)
        lift_motor_2.move_deg(LIFT_DOWN_ANGLE)
        time.sleep(1) # Give time for motors to move

        for m in motors:
            m.stop_move()
            m.disable_torque()
        if motors:
            motors[0].portHandler.closePort()
        GPIO.cleanup()
        print("Motors and GPIO cleaned up.")
