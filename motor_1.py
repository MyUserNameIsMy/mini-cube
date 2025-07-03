import sys
import tty
import termios
import select
import time
from minicubebase import MotorV1

DEVICE_NAME = '/dev/ttyUSB0' 
MOTOR_ID = 5
DXL_MINIMUM_POSITION_VALUE = 0
DXL_MAXIMUM_POSITION_VALUE = 4095 # For MX-28

# === Terminal key handling ===
def get_key():
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

def degrees_to_dxl_position(degrees):
    degrees = max(0, min(360, degrees))
    return int((degrees / 360.0) * DXL_MAXIMUM_POSITION_VALUE)

# === Main Program ===
old_settings = set_terminal_raw()
motor = None
try:
    print(f"Initializing motor ID {MOTOR_ID} on port {DEVICE_NAME}...")
    motor = MotorV1(DEVICE_NAME, MOTOR_ID)
    
    motor.set_mode('JOINT_MODE')
    print("Motor initialized successfully in JOINT_MODE.")
    
    print("\n--- MX-28 Joint Control ---")
    print("Press key for desired position:")
    print("  [0] ->   0 degrees")
    print("  [9] ->  90 degrees")
    print("  [1] -> 180 degrees")
    print("  [3] -> 360 degrees")
    print("\n  [q] -> Quit and disable motor")

    while True:
        key = get_key()

        if key in ['0', '9', '1', '3']:
            target_angle = 0
            if key == '0':
                target_angle = 0
            elif key == '9':
                target_angle = 90
            elif key == '1':
                target_angle = 180
            elif key == '3':
                target_angle = 360
            
            goal_position = degrees_to_dxl_position(target_angle)
            print(f"Moving to {target_angle}° (Position: {goal_position})...")
            # Using the set_deg method from your class
            motor.set_deg(goal_position)

        elif key == 'q':
            print("Exiting program.")
            break
        
        time.sleep(0.05)

finally:
    restore_terminal_settings(old_settings)
    
    if motor:
        print("Disabling torque and closing port...")
        present_pos = motor.get_present_position()
        print(f"Final motor position: {present_pos}")
        motor.disable_torque()
        motor.portHandler.closePort()
        print("Cleanup complete.")


