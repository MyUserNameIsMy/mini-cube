import RPi.GPIO as GPIO
import time

# --- Configuration ---
SERVO_PIN = 12

# --- PWM Setup ---
GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN, GPIO.OUT)

# Start PWM at 50Hz for the servo
pwm = GPIO.PWM(SERVO_PIN, 50) 
pwm.start(0) 

# --- Control Function ---

def set_angle_120(angle):
    """
    Sets the servo to a specific angle, respecting a 120-degree limit.
    
    :param angle: The desired angle between 0 and 120 degrees.
    """
    # --- Safety Check ---
    # Clamp the angle to the servo's physical limits (0-120)
    if angle < 0:
        angle = 0
        print("Angle too low, setting to 0 degrees.")
    if angle > 120:
        angle = 120
        print("Angle too high, setting to 120 degrees.")
    
    # Map angle (0-180 scale) to duty cycle (2% to 12%)
    # We still use the 180-degree formula because the servo's electronics
    # respond to the same pulse widths, even if its movement is limited.
    duty_cycle = 2 + (angle / 18)
    
    pwm.ChangeDutyCycle(duty_cycle)
    
    # Wait for the servo to settle at the new position
    time.sleep(1) 
    
    # Stop sending the signal to prevent servo jitter/hum
    pwm.ChangeDutyCycle(0)

# --- Main Program Loop ---
try:
    print("Controlling 120-degree positional servo. Press Ctrl+C to exit.")
    
    print("Moving to 0 degrees (minimum position)...")
    set_angle_120(0)

    print("Moving to 60 degrees (center position)...")
    set_angle_120(60)
    
    print("Moving to 120 degrees (maximum position)...")
    set_angle_120(120)
    
    # Example of the safety clamp in action
    print("\nAttempting to move to 180 degrees (should be clamped to 120)...")
    set_angle_120(180)


except KeyboardInterrupt:
    print("\nProgram stopped by user.")

finally:
    # Clean up the GPIO pins before exiting
    pwm.stop()
    GPIO.cleanup()
    print("GPIO cleanup complete.")
