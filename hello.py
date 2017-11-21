""" It runs on the microbit and repeatedly reads the compass. """

# pylint: disable=import-error

import microbit as mb  # type: ignore

mb.compass.calibrate()

while True:
    direction = mb.compass.heading()
    acceleration = mb.accelerometer.get_values()
    print(str(direction) + ' ' + str(acceleration))
    mb.sleep(100)
