# pylint: disable=missing-docstring, import-error
# There isn't enough room for docstrings on the microbit.

from typing import Dict, Set, Tuple  # noqa: F401
import microbit as mb  # type: ignore
import microbit_types as t  # noqa: F401


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


def send_output(output):  # type: (t.OutputSlashState) -> None
    op = '{} {} {} {} {} {}'.format(
        output['direction'],
        output['acceleration'][0],
        output['acceleration'][1],
        output['acceleration'][2],
        int(output['buttonA']),
        int(output['buttonB']))
    print(op)
    mb.display.clear()
    for led in output['display']:
        mb.display.set_pixel(led[0], led[1], 9)


def read_input():  # type: () -> t.RawInput
    return {
        'display': mb.uart.readall(),
        'compass_heading': mb.compass.heading(),
        'acceleration': mb.accelerometer.get_values(),
        'buttonA': mb.button_a.was_pressed(),
        'buttonB': mb.button_b.was_pressed()}


ALL_LEDS = {
    (x, y) for x in range(5) for y in range(5)}  # type: Set[Tuple[int, int]]

ERROR_LEDS = {0: set(), 1: ALL_LEDS}  # type: Dict[int, Set[Tuple[int, int]]]

RECORDING_LEDS = {
    0: set(),
    1: {(1, 2)},
    2: {v for _, v in DIRECTION_LED.items()}
    }  # type: Dict[int, Set[Tuple[int, int]]]

NEW_DESTINATION_LEDS = {
    0: set(),
    1: {(3, 2)}}  # type: Dict[int, Set[Tuple[int, int]]]


def parse_input(
        raw_input_data):  # type: (t.RawInput) -> t.ParsedInput
    no_serial_input = False  # type: bool
    recording_state_dont_care = False  # type: bool
    if raw_input_data['display'] is not None:
        as_int = int(raw_input_data['display'][-1]) & 255
        new_destination = as_int & 0b1
        direction = (as_int >> 1) & 0b1111
        recording_state = (as_int >> 5) & 0b11
        if recording_state == 3:
            recording_state_dont_care = True
        error_state = (as_int >> 7) & 0b1
    else:
        no_serial_input = True
    if no_serial_input or recording_state_dont_care:
        # mb.display.scroll('nsi' + str(no_serial_input))
        # mb.display.scroll('rsdc' + str(recording_state_dont_care))
        # error_state = 1
        error_state = 0
        direction = 0
        recording_state = 0
        new_destination = 0
    # mb.display.scroll(str(raw_input_data['compass_heading']))
    # direction_float = float(raw_input_data['compass_heading'])*2*math.pi/360
    # mb.display.scroll(str(direction_float))
    # mb.display.scroll('es' + str(error_state))
    return (
        {'display': ({DIRECTION_LED[direction]} |
                     ERROR_LEDS[error_state] |
                     RECORDING_LEDS[recording_state] |
                     NEW_DESTINATION_LEDS[new_destination]),
         'direction': raw_input_data['compass_heading'],
         'acceleration': raw_input_data['acceleration'],
         'buttonA': raw_input_data['buttonA'],
         'buttonB': raw_input_data['buttonB']})


def update_state(parsed_input):  # type: (t.ParsedInput) -> t.OutputSlashState
    return {
        'display': parsed_input['display'],
        'direction': parsed_input['direction'],
        'acceleration': parsed_input['acceleration'],
        'buttonA': parsed_input['buttonA'],
        'buttonB': parsed_input['buttonB']}


def main():
    mb.compass.calibrate()
    state = {
        'display': [],
        'direction': 0,
        'acceleration': (0, 0, 0),
        'buttonA': False,
        'buttonB': False}  # type: t.OutputSlashState
    while True:
        send_output(state)
        raw_input_data = read_input()  # type: Tuple[str, t.RawInput]
        state = update_state(
            parse_input(raw_input_data))  # type: OutputSlashState
        mb.sleep(600)


main()
