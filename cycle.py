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
HALL_Y_PINS = {'front': 4, 'back': 7}
HALL_X_PINS = {'left': 5, 'right': 8}
MAGNET_PIN = 6

# --- Lift Motor Angles (in degrees) ---
LIFT_UP_ANGLE = 2060
LIFT_DOWN_ANGLE = -2060
Z_AXIS_HOME_POSITION_DEG = 18000
Z_AXIS_PICKUP_DEG = -14000  # Relative movement for pickup/place

# === Motor Initialization ===
# In a real scenario, these lines would initialize the actual motors.
motor1_z = MotorV1(DEVICE_NAME, 4)
motor1_x = MotorV1(DEVICE_NAME, 1)
motor2_x = MotorV1(DEVICE_NAME, 3)
motor1_y = MotorV2(DEVICE_NAME, 1)
motor2_y = MotorV2(DEVICE_NAME, 2)
lift_motor_1 = MotorV1(DEVICE_NAME, 5)
lift_motor_2 = MotorV1(DEVICE_NAME, 7)

# Consolidate all motors for easier management
all_motors = {
    'z': motor1_z, 'x1': motor1_x, 'x2': motor2_x,
    'y1': motor1_y, 'y2': motor2_y, 'lift1': lift_motor_1, 'lift2': lift_motor_2
}


# === Main Robot Controller Class ===
class RobotController:
    """Manages the robot's state and high-level actions."""

    def __init__(self, motors, initial_grid, start_pos):
        self.motors = motors
        self.grid = [row[:] for row in initial_grid]  # Create a copy
        # Robot's current position (using 0-based indexing: 0, 1, 2)
        self.current_pos = start_pos
        self.setup_hardware()
        self.lift_wall = 'UP'

    def setup_hardware(self):
        """Initializes GPIO and motor settings."""
        print("--- Initializing Hardware ---")
        GPIO.cleanup()
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(MAGNET_PIN, GPIO.OUT, initial=GPIO.LOW)
        for pin in list(HALL_Y_PINS.values()) + list(HALL_X_PINS.values()):
            GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # Set speeds and enable torque
        self.motors['z'].set_speed(400)
        self.motors['x1'].set_speed(1000)
        self.motors['x2'].set_speed(1000)
        self.motors['y1'].set_speed(1000)
        self.motors['y2'].set_speed(1000)

        for motor in self.motors.values():
            motor.enable_torque()

        # Set motor operating modes
        self.motors['x1'].set_mode('MULTI_TURN_MODE')
        self.motors['x2'].set_mode('MULTI_TURN_MODE')
        self.motors['y1'].set_mode('EXTENDED_POSITION_MODE')
        self.motors['y2'].set_mode('EXTENDED_POSITION_MODE')
        self.motors['z'].set_mode('MULTI_TURN_MODE')
        self.motors['lift1'].set_mode('MULTI_TURN_MODE')
        self.motors['lift2'].set_mode('MULTI_TURN_MODE')
        print("--- Hardware Initialized ---\n")

    def control_magnet(self, state):
        """Turns the magnet ON or OFF."""
        print(f"-> Turning magnet {state.upper()}")
        GPIO.output(MAGNET_PIN, GPIO.HIGH if state.upper() == 'ON' else GPIO.LOW)
        time.sleep(1)

    def move_z_axis(self, position):
        """Moves the Z-axis to a specified position."""
        print(f"-> Moving Z-axis to {position}")
        deg = Z_AXIS_HOME_POSITION_DEG if position == 'HOME' else Z_AXIS_PICKUP_DEG
        self.motors['z'].set_deg(deg) if position == 'HOME' else self.motors['z'].move_deg(deg)
        time.sleep(5)  # Wait for movement to complete

    def lift_gantry(self, position):
        """Lifts or lowers the entire gantry."""
        print(f"-> Moving gantry {position.upper()}")
        angle = LIFT_UP_ANGLE if position.upper() == 'UP' else LIFT_DOWN_ANGLE
        self.motors['lift1'].move_deg(angle)
        self.motors['lift2'].move_deg(angle)
        time.sleep(2)

    def _move_along_axis(self, axis, direction):
        """Generic internal function to move one cell along X or Y."""
        print(f"--> Moving one cell {direction} along {axis.upper()}-axis")
        if direction == 'x':
            self.lift_gantry('UP')
            self.lift_wall = 'UP'
        else:
            self.lift_gantry('DOWN')
            self.lift_wall = 'DOWN'

        m1 = self.motors[f'{axis}1']
        m2 = self.motors[f'{axis}2']
        if axis == 'x':
            if direction == 'FORWARD':
                hall_pin = HALL_X_PINS['left']
            else:
                hall_pin = HALL_Y_PINS['right']
        else:
            if direction == 'FORWARD':
                hall_pin = HALL_Y_PINS['back']
            else:
                hall_pin = HALL_X_PINS['front']


        if axis == 'x':
            m1.set_mode('WHEEL_MODE')
            m2.set_mode('WHEEL_MODE')
        else:
            m1.set_mode('VELOCITY_MODE')
            m2.set_mode('VELOCITY_MODE')
        time.sleep(0.05)

        if direction == 'FORWARD':
            m1.move_backward()
            m2.move_forward()
        else:  # BACKWARD
            m1.move_forward()
            m2.move_backward()
        sleep_time = 1.5 if direction == 'FORWARD' and axis == 'x' else 0.5
        # Simplified movement logic for clarity
        time.sleep(sleep_time)
        # In a real scenario, you would use hall sensor feedback here
        while GPIO.input(hall_pin) == GPIO.HIGH: time.sleep(0.05)

        m1.stop_move()
        m2.stop_move()
        time.sleep(0.5)

        if axis == 'y' and direction == 'FORWARD':
            m1.set_mode('EXTENDED_POSITION_MODE')
            m2.set_mode('EXTENDED_POSITION_MODE')
            time.sleep(0.05)
            m1.move_deg(-3100)
            m2.move_deg(3100)
            time.sleep(1)
            m1.stop_move()
            m2.stop_move()

        if axis == 'x' and direction == 'BACKWARD':
            m1.set_mode('MULTI_TURN_MODE')
            m2.set_mode('MULTI_TURN_MODE')
            time.sleep(0.05)
            m1.move_deg(1750)
            m2.move_deg(-1750)
            time.sleep(1)
            m1.stop_move()
            m2.stop_move()

    def go_to(self, target_x, target_y):
        """Moves the robot from its current position to a target grid cell."""
        print(f"-> Navigating from {self.current_pos} to ({target_x}, {target_y})")
        # Move along Y-axis
        while self.current_pos['y'] < target_y:
            self._move_along_axis('y', 'BACKWARD')
            self.current_pos['y'] += 1
        while self.current_pos['y'] > target_y:
            self._move_along_axis('y', 'FORWARD')
            self.current_pos['y'] -= 1

        # Move along X-axis
        while self.current_pos['x'] < target_x:
            self._move_along_axis('x', 'FORWARD')
            self.current_pos['x'] += 1
        while self.current_pos['x'] > target_x:
            self._move_along_axis('x', 'BACKWARD')
            self.current_pos['x'] -= 1
        print(f"-> Arrived at {self.current_pos}")

    def pick_up(self):
        """Executes the sequence to pick up a box."""
        print("-> Executing PICK UP sequence")
        self.move_z_axis('PICKUP')
        self.control_magnet('ON')
        self.move_z_axis('HOME')

    def place_down(self):
        """Executes the sequence to place a box."""
        print("-> Executing PLACE DOWN sequence")
        self.move_z_axis('PICKUP')
        self.control_magnet('OFF')
        self.move_z_axis('HOME')

    def move_box(self, from_pos, to_pos):
        """High-level function to move a single box from one cell to another."""
        fx, fy = from_pos
        tx, ty = to_pos
        print(f"\n=== Moving box from ({fx},{fy}) to ({tx},{ty}) ===")

        # Go to source, pick up, go to destination, place down
        self.go_to(fx, fy)
        self.pick_up()
        self.grid[fy][fx] -= 1  # Update grid state

        self.go_to(tx, ty)
        self.place_down()
        self.grid[ty][tx] += 1  # Update grid state
        print(f"Grid state: {self.grid}")

    def unstack_and_move_bottom_box(self, stack_pos, target_pos, temp_pos):
        """
        Moves the bottom box from a stack to a target location by using a
        temporary holding spot for the boxes on top.
        """
        stack_x, stack_y = stack_pos
        num_boxes_to_move = self.grid[stack_y][stack_x] - 1

        if num_boxes_to_move < 0:
            print(f"Error: No boxes to move at {stack_pos}")
            return

        print(f"\n=== Unstacking {num_boxes_to_move} boxes from ({stack_x},{stack_y}) to {temp_pos} ===")
        # 1. Unstack the top boxes and move them to a temporary location
        for i in range(num_boxes_to_move):
            self.move_box(from_pos=stack_pos, to_pos=temp_pos)

        # 2. Move the now-exposed bottom box to its final target
        print(f"\n=== Moving BOTTOM box from {stack_pos} to {target_pos} ===")
        self.move_box(from_pos=stack_pos, to_pos=target_pos)

        # 3. Restack the boxes from the temporary location back to the original stack
        print(f"\n=== Restacking {num_boxes_to_move} boxes from {temp_pos} to {stack_pos} ===")
        for i in range(num_boxes_to_move):
            self.move_box(from_pos=temp_pos, to_pos=stack_pos)

    def cleanup(self):
        """Disables motors and cleans up GPIO resources."""
        print("\nExiting and cleaning up...")
        for motor in self.motors.values():
            motor.stop_move()
            motor.disable_torque()
            time.sleep(0.05)
            if motor.portHandler.is_open:
                motor.portHandler.closePort()
        GPIO.cleanup()
        print("Motors and GPIO cleaned up successfully.")


# === Main Execution Logic ===
def solve_puzzle():
    """
    This function defines the high-level logic to solve the puzzle.
    It uses the RobotController to abstract away the complex movements.
    """
    # --- UPDATED: Grid state to match the user's diagram ---
    # Coordinate system: grid[y][x] where (0,0) is TOP-LEFT
    initial_grid = [
        [0, 0, 3],  # Row 0: 3 boxes at (x=2, y=0) -> "33" in user's diagram
        [0, 0, 0],  # Row 1
        [1, 0, 0]  # Row 2: 1 box at (x=0, y=2) -> "11" in user's diagram
    ]

    # --- UPDATED: Key locations based on diagram ---
    # (x, y) coordinates based on a 0-indexed grid
    pos_11 = (0, 2)  # Bottom-Left
    pos_33 = (2, 0)  # Top-Right
    pos_13 = (0, 0)  # Top-Left (as per user instruction "move 11 to 13")

    # Define a temporary holding spot for unstacking. (x=1, y=0) is Top-Middle.
    temp_pos_for_33 = (1, 0)

    # --- UPDATED: Robot's starting position ---
    start_pos = {'x': 0, 'y': 2}  # Start at "11" (bottom-left)

    # Initialize the controller with the new configuration
    robot = RobotController(all_motors, initial_grid, start_pos)

    try:
        input("Press ENTER to begin the puzzle sequence...")

        # Your sequence, translated into clear actions with the correct coordinates:
        # 1. "move 11 box to 13"
        robot.move_box(from_pos=pos_11, to_pos=pos_13)

        # 2. "get last box from grid 33 and move it to 11"
        # This requires unstacking the top two boxes first.
        robot.unstack_and_move_bottom_box(stack_pos=pos_33, target_pos=pos_11, temp_pos=temp_pos_for_33)

        # 3. "and then 11 to 33"
        robot.move_box(from_pos=pos_11, to_pos=pos_33)

        # 4. "and 13 back to 11"
        robot.move_box(from_pos=pos_13, to_pos=pos_11)

        print("\n--- PUZZLE COMPLETE! ---")

    except KeyboardInterrupt:
        print("\nProgram interrupted by user.")
    finally:
        robot.cleanup()


if __name__ == "__main__":
    solve_puzzle()
