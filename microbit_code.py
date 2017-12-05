import math

import microbit as mb
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

def send_output(output):
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

def state2output(state):
    return {
        'display': state['display'],
        'direction': state['direction'],
        'acceleration': state['acceleration'],
        'buttonA': state['buttonA'],
        'buttonB': state['buttonB']}

def read_input():
    return {
        'display': mb.uart.readall(),
        'compass_heading': mb.compass.heading(),
        'acceleration': mb.accelerometer.get_values(),
        'buttonA': mb.button_a.was_pressed(),
        'buttonB': mb.button_b.was_pressed()}

ALL_LEDS = {(x, y) for x in range(5) for y in range(5)}

ERROR_LEDS = {0: set(), 1: ALL_LEDS}

RECORDING_LEDS = {
    0: set(),
    1: {(1, 2)},
    2: {v for _, v in DIRECTION_LED.items()}}

NEW_DESTINATION_LEDS = {
    0: set(),
    1: {(3, 2)}}

def parse_input(raw_input_data):
    if raw_input_data is None:
        return {
            'errors': {'serial_input': 'No input.'},
            'display': None,
            'direction': None,
            'acceleration': None,
            'buttonA': None,
            'buttonB': None}
    as_int = int(raw_input_data) & 255
    new_destination = as_int & 0b1
    direction = (as_int >> 1) & 0b1111
    recording_state = (as_int >> 5) & 0b11
    if recording_state == 3:
        return ("The spare bit in the recording state was used and "
                "shouldn't have been.", None)
    error_state = (as_int >> 7) & 0b1
    return {
        'errors': {'serial_input': None},
        'display': ({DIRECTION_LED[direction]} |
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
        'buttonB': raw_input_data['buttonB']}


def update_state(parsed_input):
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
        'acceleration': {'x': 0, 'y': 0, 'z': 0},
        'buttonA': False,
        'buttonB': False}
    while True:
        send_output(state2output(state))
        raw_input_data = read_input()
        state = update_state(parse_input(raw_input_data))

main()
