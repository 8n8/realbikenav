""" The code to run on the microbit. """

# pylint: disable=import-error

import microbit as mb  # type: ignore


led = mb.display.set_pixel


# The direction LEDs are the ones at the edge of the 5 x 5 LED block.  They
# are numbered as follows:
#
#               X
#       |0  |1  |2  |3  |4
#    ---|-------------------
#    0  |14 |15 |0  |1  |2
#    1  |13 |   |   |   |3
# Y  2  |12 |   |   |   |4
#    3  |11 |   |   |   |5
#    4  |10 |9  |8  |7  |6
DIRECTION_LED = {
    0: (2, 0),
    1: (3, 0),
    2: (4, 0),
    3: (4, 1),
    4: (4, 2),
    5: (4, 3),
    6: (4, 4),
    7: (3, 4),
    8: (2, 4),
    9: (1, 4),
    10: (0, 4),
    11: (0, 3),
    12: (0, 2),
    13: (0, 1),
    14: (0, 0),
    15: (1, 0)}


def set_centre_LEDs(brightness):
    """ It sets the brightness of the nine centre LEDs. """
    for x in (1, 2, 3):
        for y in (1, 2, 3):
            led(x, y, brightness)


def flash_centre():
    """ It flashes the nine centre LEDs on and off again. """
    set_centre_LEDs(9)
    mb.sleep(500)
    set_centre_LEDs(0)


def display(signal: bytes):
    """
    It displays the signal.

    The signal is contained in one byte.  The bits are allocated as
    follows:

    12345678
    ________
    ^^^^^^^^
    |||||||new destination just created or not
    |||the direction to drive in - four bits because 16 LEDs on edge
    |current recording state - on or two kinds of off
    error state - Ok or not

    The signal codes are:

        + new destination just created
          1 is yes and 0 is no
        + recording state
          0 is not recording because paused
          1 is recording
          2 is not recording because of arrival at destination
        + direction is encoded as in the comment for DIRECTION_LED
        + error state
          0 is no errors
          1 is errors
    """
    as_int = int(signal)
    if as_int > 255 or as_int < 0:
        return 'The number was out of range.'
    new_destination = as_int & 0b1
    direction = (as_int >> 1) & 0b1111
    recording_state = (as_int >> 5) & 0b11
    error_state = (as_int >> 7) & 0b1

    mb.display.clear()
    if new_destination:
        flash_centre()

    dirx, diry = DIRECTION_LED[direction]
    led(dirx, diry, 9)

    if recording_state == 1:
        led(1, 2, 9)

    if recording_state == 2:
        for x, y in DIRECTION_LED.values():
            led(x, y, 9)

    if error_state:
        for x in range(4):
            for y in range(4):
                led(x, y, 9)


def main():
    """
    A continuous loop that communicates with the laptop and updates the
    microbit LED display.
    """
    mb.compass.calibrate()
    button_on = False
    while True:
        signal = mb.uart.readall()
        display(signal)
        direction = mb.compass.heading()
        acceleration = mb.accelerometer.get_values()
        button_reading = mb.button_a.was_pressed()
        button_on = not button_reading == button_on
        mb.display.set_pixel(0, 2, button_on * 9)
        print('{} {} {} {}'.format(
            direction, acceleration, button_on, mb.button_b.was_pressed()))
        mb.sleep(200)


main()
