""" It runs on the microbit and repeatedly reads the compass. """

# pylint: disable=import-error

import microbit as mb  # type: ignore

mb.compass.calibrate()

button_on = False

mb.display.set_pixel(4, 4, 9)

while True:
    direction = mb.compass.heading()
    acceleration = mb.accelerometer.get_values()
    button_reading = mb.button_a.was_pressed()
    button_on = not button_reading == button_on
    mb.display.set_pixel(0, 2, button_on * 9)
    print('{} {} {}'.format(direction, acceleration, button_on))
    mb.sleep(200)
