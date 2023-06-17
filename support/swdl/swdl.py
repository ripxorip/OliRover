# -*- coding: utf-8 -*-
# vim:fenc=utf-8
import os
import sys
import time

# Import raspberry pi GPIO library
import RPi.GPIO as GPIO

# main function
def main():
    # Read file from argv
    binary = sys.argv[1]
    print("Binary: " + binary)

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)

    # Reset the board by setting GPIO 4 to high
    GPIO.setup(4, GPIO.OUT)
    GPIO.output(4, GPIO.LOW)

    # Set the boot pin to high (GPIO 26)
    GPIO.setup(26, GPIO.OUT)
    GPIO.output(26, GPIO.HIGH)

    # Wait one second
    time.sleep(1)
    GPIO.output(4, GPIO.HIGH)
    GPIO.output(26, GPIO.LOW)

    # Download the binary
    os.system(f"/home/ripxorip/dev/OliRover/support/swdl/pico-py-serial-flash/.venv/bin/python /home/ripxorip/dev/OliRover/support/swdl/pico-py-serial-flash/main.py /dev/ttyS0 {binary}")

if __name__ == "__main__":
    main()
