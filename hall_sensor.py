import RPi.GPIO as GPIO
import time

HALL_SENSOR_PIN = 7

try:
    # Set the GPIO mode and setup the pin
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(HALL_SENSOR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    print(f"Monitoring Hall Sensor on GPIO {HALL_SENSOR_PIN}. Press Ctrl+C to exit.")

    last_state = None

    # Main loop to read the sensor
    while True:
        current_state = GPIO.input(HALL_SENSOR_PIN)

        # Report state change only
        if current_state != last_state:
            if current_state == GPIO.LOW:
                print("Magnet Detected")
            else:
                print("No Magnet")
            
            last_state = current_state

        time.sleep(0.1)

except KeyboardInterrupt:
    # Handle Ctrl+C to exit gracefully
    print("\nProgram stopped by user.")

finally:
    # Clean up GPIO pins
    GPIO.cleanup()
    print("GPIO cleanup complete.")

