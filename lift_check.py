import sys
import time
from minicubebase import MotorV1

# === Motor and Port Setup ===
DEVICE_NAME = '/dev/ttyUSB0'

# Initialize the two lifting motors you want to test
try:
    lift_motor_1 = MotorV1(DEVICE_NAME, 5)
    lift_motor_2 = MotorV1(DEVICE_NAME, 7)
except Exception as e:
    print(f"Failed to initialize motors: {e}")
    print("Please ensure the motors are connected and the device name is correct.")
    sys.exit(1)


# === Main Program ===
if __name__ == "__main__":
    print("--- Lift Motor Angle Calibration ---")

    try:
        # Enable torque and set to joint mode
        lift_motor_1.enable_torque()
        lift_motor_2.enable_torque()
        lift_motor_1.set_mode('JOINT_MODE')
        lift_motor_2.set_mode('JOINT_MODE')

        print("Motors initialized in JOINT_MODE.")
        print("Enter an angle (0-300) to move the motors.")
        print("Type 'q' or 'quit' to exit.")

        while True:
            # Get user input
            try:
                user_input = input("\nEnter angle (0-300) or 'q' to quit: ")
            except EOFError: # Handle case where input stream is closed
                break

            # Check for exit condition
            if user_input.lower() in ['q', 'quit']:
                print("Exiting program.")
                break

            # Validate and convert input to an integer
            try:
                angle = int(user_input)
            except ValueError:
                print("Error: Invalid input. Please enter a number.")
                continue

            # Move the motors to the specified angle
            print(f"Moving motors to {angle}°...")
            lift_motor_1.set_deg(angle)
            lift_motor_2.set_deg(angle)
            print("Move command sent.")

    except KeyboardInterrupt:
        print("\nProgram interrupted by user.")
    finally:
        # Graceful shutdown
        print("Disabling motor torque and closing port...")
        try:
            lift_motor_1.disable_torque()
            lift_motor_2.disable_torque()
            # The port is shared, so we only need to close it once.
            lift_motor_1.portHandler.closePort()
            print("Cleanup complete.")
        except Exception as e:
            print(f"An error occurred during cleanup: {e}")
