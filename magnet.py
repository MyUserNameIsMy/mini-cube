# Import the necessary libraries
import RPi.GPIO as GPIO
import time

# Set the GPIO numbering mode to BCM
GPIO.setmode(GPIO.BCM)

# Define the GPIO pin that the component is connected to
MAGNET = 6

# --- Correction ---
# Set up the MAGNET pin as an output pin
# We also set its initial state to LOW (off)
GPIO.setup(MAGNET, GPIO.OUT, initial=GPIO.LOW)


def main():
    """
    Main function to control the GPIO pin based on keyboard input.
    """
    print("GPIO Control Initialized.")
    print("Press 't' to turn ON, 'o' to turn OFF, 'q' to quit.")

    try:
        while True:
            # Wait for user input
            choice = input("Enter your choice: ")

            if choice.lower() == 't':
                # Turn the component ON by setting the GPIO pin to HIGH
                GPIO.output(MAGNET, GPIO.HIGH)
                print("MAGNET -> ON")

            elif choice.lower() == 'o':
                # Turn the component OFF by setting the GPIO pin to LOW
                GPIO.output(MAGNET, GPIO.LOW)
                print("MAGNET -> OFF")

            elif choice.lower() == 'q':
                # Break the loop and exit the program
                print("Exiting program.")
                break

            else:
                print("Invalid input. Please use 't', 'o', or 'q'.")

    except KeyboardInterrupt:
        # This will also trigger the cleanup
        print("Program interrupted.")

    finally:
        # This block will run no matter how the try block exits
        # It ensures that the GPIO pins are reset to their default state
        GPIO.cleanup()
        print("GPIO cleanup complete. Goodbye.")


if __name__ == '__main__':
    main()