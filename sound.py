"""
This Raspberry Pi Pico MicroPython code was developed by newbiely.com
This Raspberry Pi Pico code is made available for public use without any restriction
For comprehensive instructions and wiring diagrams, please visit:
https://newbiely.com/tutorials/raspberry-pico/raspberry-pi-pico-sound-sensor
"""

from machine import Pin
import time

SENSOR_PIN = 0 # The Raspberry Pi Pico pin (GP0) connected to OUT pin of the sound sensor
prev_sound_state = 1  # the previous state from the input pin
sound_state = 1  # the current reading from the input pin
# Initialize the sensor pin as an input
sensor_pin = Pin(SENSOR_PIN, Pin.IN)

# Main loop
while True:
    # Read the state of the input pin
    sound_state = sensor_pin.value()

    if prev_sound_state == 1 and sound_state == 0:
        print("The sound has been detected")
    elif prev_sound_state == 0 and sound_state == 1:
        print("The sound has disappeared")

    # Save the last state
    prev_sound_state = sound_state

    time.sleep(0.1)  # Delay for 100 milliseconds to mimic Arduino's loop delay
