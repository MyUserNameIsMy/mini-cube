import sys
import tty
import termios
import select
import time
from gpiozero import AngularServo, LED
from minicubebase import MotorV1, MotorV2

# === gpiozero Setup ===
# The AngularServo handles all PWM and cleanup.
# We configure it for a standaard 180-degree servo.
# Change max_angle to 120 if your servo is physically limited.
servo1 = AngularServo(12, min_angle=0, max_angle=120, min_pulse_width=0.0009, max_pulse_width=0.003)
servo2 = AngularServo(13, min_angle=0, max_angle=180, min_pulse_width=0.0005, max_pulse_width=0.0025)

# Using the LED class for the magnet provides a simple .on()/.off() interface
magnet = LED(6)

# === Motor Setup ===
DEVICE_NAME = '/dev/ttyUSB0'
motor1_z = MotorV1(DEVICE_NAME, 4)
motor1_x = MotorV1(DEVICE_NAME, 1)
motor2_x = MotorV1(DEVICE_NAME, 3)
motor1_y = MotorV2(DEVICE_NAME, 1)
motor2_y = MotorV2(DEVICE_NAME, 2)

motors = [motor1_z, motor1_x, motor1_y, motor2_x, motor2_y]
for m in motors:
    m.enable_torque()

motor1_x.set_mode('WHEEL_MODE')
motor1_x.set_mode('WHEEL_MODE')
motor2_y.set_mode('VELOCITY_MODE')
motor2_y.set_mode('VELOCITY_MODE')
motor1_z.set_mode('WHEEL_MODE')

print("🕹️  Arrow ↑ selects motors 1 & 2 | ↓ selects motors 3 & 4")
print("    Use [w] = forward | [s] = backward | [q] = quit")
print("    Use [t] = magnet ON | [o] = magnet OFF")

# === Motor selection ===
selected = [motor1_x, motor2_x]

# === Terminal key handling (unchanged) ===
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
old_settings = set_terminal_raw()
try:
    while True:
        key = get_key()

        if key == '\x1b[A':  # Up Arrow
            selected = [motor1_x, motor2_x]
            print("↑ Selected motors 1 & 2 | Setting servos to 110° & 95°")
            servo1.angle = 65
            servo2.angle = 95
        elif key == '\x1b[B':  # Down Arrow
            selected = [motor1_y, motor2_y]
            print("↓ Selected motors 3 & 4 | Setting servos to 0°")
            servo1.angle = 0
            servo2.angle = 0
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
            magnet.on()
            print("MAGNET -> ON")
        elif key == 'o':
            magnet.off()
            print("MAGNET -> OFF")
        elif key == 'q':
            print("Exiting...")
            break
        elif key is not None:
            # Stop motors on any other key press
            motor1_z.stop_move()
            for m in selected:
                m.stop_move()
        
        time.sleep(0.05)

finally:
    # --- Cleanup ---
    restore_terminal_settings(old_settings)
    
    # Stop all motors from minicubebase
    for m in motors:
        m.stop_move()
        m.disable_torque()
        m.portHandler.closePort()
        
    # Detach servos and turn off magnet (handled by gpiozero)
    servo1.detach()
    servo2.detach()
    magnet.close()
    
    print("Motors and servos stopped, GPIO resources released.")
