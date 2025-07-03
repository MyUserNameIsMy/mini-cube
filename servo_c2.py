from gpiozero import AngularServo
from time import sleep

# --- Configuration ---
# The GPIO pin connected to the servo's signal wire
SERVO_PIN = 12

# --- Servo Setup ---
# We configure the servo with its physical limits (0 to 120 degrees).
# The pulse widths are typical values for an MG996R. You may need to
# fine-tune them if the servo doesn't reach the full 0 or 120 angle.
servo = AngularServo(
    SERVO_PIN,
    min_angle=0,
    max_angle=120,
    min_pulse_width=0.0004,  # 0.5 ms
    max_pulse_width=0.0025  # 2.5 ms
)

# --- Main Program Loop ---
try:

    while True:

        print("Controlling 120-degree servo with gpiozero. Press Ctrl+C to exit.")
    
        print("Moving to 0 degrees (minimum)...")
        servo.angle = 0
        sleep(2)

        print("Moving to 60 degrees (center)...")
        servo.angle = 68
        sleep(2)
    
        print("Moving to 120 degrees (maximum)...")
        servo.angle = 120
        sleep(2)

        # Example of the safety limit in action.
        # gpiozero will raise an error if you try to go beyond the set limits.
        print("\nReturning to center...")
        servo.angle = 68
        sleep(2)


except KeyboardInterrupt:
    print("\nProgram stopped by user.")

finally:
    # Detach the servo to stop sending signals
    servo.detach()
    print("Servo detached and cleanup complete.")
