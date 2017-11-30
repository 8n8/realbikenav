import microbit as m

m.uart.init(baudrate=115200)


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


def flash_centre():
    led(1, 1, 9)
    led(1, 2, 9)
    led(1, 3, 9)
    led(2, 1, 9)
    led(2, 2, 9)
    led(2, 3, 9)
    led(3, 1, 9)
    led(3, 2, 9)
    led(3, 3, 9)
    sleep(1000)
    led(1, 1, 0)
    led(1, 2, 0)
    led(1, 3, 0)
    led(2, 1, 0)
    led(2, 2, 0)
    led(2, 3, 0)
    led(3, 1, 0)
    led(3, 2, 0)
    led(3, 3, 0)


CENTRE_LED_COORDS = [
    (1, 1), (2, 1), (3, 1), (1, 2), (2, 2), (3, 2), (1, 3), (2, 3), (3, 3)]


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
    if signal > 255 or signal < 0:
        return 'The number was out of range.'
    new_destination = as_int & 0b1
    direction = (as_int >> 1) & 0b1111
    recording_state = (as_int >> 5) & 0b11
    error_state = (as_int >> 7) & 0b1

    m.display.clear()
    if new_destination:
        flash_centre()

    dirx, diry = DIRECTION_LED[direction]
    led(dirx, diry, 9)
    
    if recording_state == 1:
        led(1, 2, 9)

    if recording_state == 2:
        _ = [led(x, y, 9) for x, y in CENTRE_LED_COORDS]

    if error_state:
        _ = [led(x, y, 9) for x in range(4) for y in range(4)]



while True:
    signal = m.uart.readall()
    if signal is not None:
        display(signal)
        m.display.set_pixel(2, 2, 9)
        m.sleep(500)
    m.display.clear()
