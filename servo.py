import RPi.GPIO as GPIO
import time

# Use GPIO 12 for hardware PWM (recommended)
SERVO_PIN = 12

# Set up GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN, GPIO.OUT)

# Set up PWM (50 Hz is standard for servos)
pwm = GPIO.PWM(SERVO_PIN, 50)
pwm.start(0)

def set_angle(angle):
    """
    Moves the MG995 servo to the specified angle (0-180).
    """
    duty_cycle = 2 + (angle / 18)  # Convert angle to duty cycle
    pwm.ChangeDutyCycle(duty_cycle)
    time.sleep(0.3)  # Delay for servo to move
    pwm.ChangeDutyCycle(0)  # Stop signal to avoid jitter

try:
    while True:
        angle = int(input("Enter angle (0-180): "))
        if 0 <= angle <= 180:
            set_angle(angle)
        else:
            print("Please enter a valid angle between 0 and 180.")

except KeyboardInterrupt:
    print("\nExiting...")

finally:
    pwm.stop()
    GPIO.cleanup()
