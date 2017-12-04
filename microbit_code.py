""" The code to run on the microbit. """

# pylint: disable=import-error

import math
# from typing import Dict, Set, Tuple  # noqa: F401

import microbit as mb  # type: ignore
# from mypy_extensions import TypedDict


# class Acceleration(TypedDict):
#     """ The three acceleration components. """
#     x: int
#     y: int
#     z: int
# 
# 
# class Output(TypedDict):
#     """ It represents the data for sending to the output. """
#     display: Set[Tuple[int, int]]
#     direction: float
#     acceleration: Acceleration
#     buttonA: bool
#     buttonB: bool
# 
# 
# class State(TypedDict):
#     """ It represents the state of the loop. """
#     display: Set[Tuple[int, int]]
#     direction: float
#     acceleration: Acceleration
#     buttonA: bool
#     buttonB: bool
# 
# 
# class RawInput(TypedDict):
#     """ It represents all the raw inputs. """
#     display: bytes
#     compass_heading: int
#     acceleration: Tuple[int, int, int]
#     buttonA: bool
#     buttonB: bool
# 
# 
# class ParsedInput(TypedDict):
#     """ It represents the all the input readings after parsing. """
#     display: Set[Tuple[int, int]]
#     direction: float
#     acceleration: Acceleration
#     buttonA: bool
#     buttonB: bool


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
    15: (1, 0)}  # type: Dict[int, Tuple[int, int]]


def send_output(output):  # type: (Output) -> None
    """
    It sends the output to the parent computer, and sets the display.
    """
    print('{} {} {} {} {} {}'.format(
        output['direction'],
        output['acceleration']['x'],
        output['acceleration']['y'],
        output['acceleration']['z'],
        output['buttonA'],
        output['buttonB']))
    mb.display.clear()
    for led in output['display']:
        mb.display.set_pixel(led[0], led[1], 9)


def state2output(state):  # type: (State) -> Output
    """ Given the state, it produces the data for the output. """
    return {
        'display': state['display'],
        'direction': state['direction'],
        'acceleration': state['acceleration'],
        'buttonA': state['buttonA'],
        'buttonB': state['buttonB']}


def read_input():  # type: () -> RawInput
    """ It reads all the input devices. """
    return {
        'display': mb.uart.readall[-1],
        'compass_heading': mb.compass.heading(),
        'acceleration': mb.accelerometer.get_values(),
        'buttonA': mb.button_a.was_pressed(),
        'buttonB': mb.button_b.was_pressed()}


# A set of all the LED coordinates.
ALL_LEDS = {
    (x, y) for x in range(5) for y in range(5)}  # type: Set[Tuple[int, int]]


# A dictionary of 'error code': 'led coordinate list'.
ERROR_LEDS = {
    0: set(), 1: ALL_LEDS}  # type: Dict[int, Set[Tuple[int, int]]]


# A dictionary of 'recording state code': 'led coordinate list'.
RECORDING_LEDS = {
    0: set(),
    1: {(1, 2)},
    2: {v for _, v in DIRECTION_LED.items()}
    }  # type: Dict[int, Set[Tuple[int, int]]]


# A dictionary of 'new destination code': 'led coordinate list'.
NEW_DESTINATION_LEDS = {
    0: set(),
    1: {(3, 2)}}  # type: Dict[int, Set[Tuple[int, int]]]


def parse_input(raw_input_data):  # type: (RawInput) -> Tuple[str, ParsedInput]
    """
    It converts the signal from the main computer into a list of LED
    coordinates that should be lit up.

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
    try:
        as_int = int(raw_input_data['display'])
    except ValueError:
        return 'Could not convert to int.', None
    if as_int > 255 or as_int < 0:
        return 'Number out of range.', None
    new_destination = as_int & 0b1
    direction = (as_int >> 1) & 0b1111
    recording_state = (as_int >> 5) & 0b11
    if recording_state == 3:
        return ("The spare bit in the recording state was used and "
                "shouldn't have been.", None)
    error_state = (as_int >> 7) & 0b1
    return (
        None,
        {'display': ({DIRECTION_LED[direction]} |
                     ERROR_LEDS[error_state] |
                     RECORDING_LEDS[recording_state] |
                     NEW_DESTINATION_LEDS[new_destination]),
         'direction':
             float(raw_input_data['compass_heading'])*2*math.pi/360,
         'acceleration': {
             'x': raw_input_data['acceleration'][0],
             'y': raw_input_data['acceleration'][1],
             'z': raw_input_data['acceleration'][2]},
         'buttonA': raw_input_data['buttonA'],
         'buttonB': raw_input_data['buttonB']})


def update_state(parsed_input):  # type: (ParsedInput) -> State
    """ It creates the new state from the parsed input readings. """
    return {
        'display': parsed_input['display'],
        'direction': parsed_input['direction'],
        'acceleration': parsed_input['acceleration'],
        'buttonA': parsed_input['buttonA'],
        'buttonB': parsed_input['buttonB']}


def main():
    """
    A continuous loop that communicates with the laptop and updates the
    microbit LED display.
    """
    mb.compass.calibrate()
    state = {
        'display': [],
        'direction': 0,
        'acceleration': {'x': 0, 'y': 0, 'z': 0},
        'buttonA': False,
        'buttonB': False}  # type: State
    while True:
        send_output(state2output(state))
        raw_input_data = read_input()
        state = update_state(parse_input(raw_input_data))


main()
