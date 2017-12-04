"""
It constantly records the camera images and the corresponding microbit
readings, gps readings, and target direction from the routing algorithm.
"""

import enum
import json
import math
import os
import random
import time
from typing import Tuple

import cv2  # type: ignore
from mypy_extensions import TypedDict
import numpy as np  # noqa: F401
import parse  # type: ignore
import plan_route
import serial  # type: ignore


@enum.unique
class RecordingState(enum.Enum):
    """
    It represents the recording state.  Either the recording is going on,
    it has been paused by the user pressing a button on the microbit, or
    it has stopped because the bike has arrived at its destination.
    """
    ON = enum.auto()
    OFF_PAUSED = enum.auto()
    OFF_ARRIVED = enum.auto()


class Acceleration(TypedDict):
    """ It represents the acceleration reading. """
    x: int
    y: int
    z: int


class MicrobitReading(TypedDict):
    """ It represents one reading of the microbit sensors. """
    accel: Acceleration
    compass: int
    buttonA: bool
    buttonB: bool


class Errors(TypedDict):
    """
    It contains all the error messages arising from reading the sensors and
    calculating the route.
    """
    route: str
    gps_read: str
    gps_parse: str
    photo_ok: bool
    microbit_read: str
    microbit_parse: str


class ParsedInput(TypedDict):
    """ It contains the input data after parsing.  """
    microbit: MicrobitReading
    gps: plan_route.MapPosition
    grey_photo: 'np.ndarray[np.uint8]'
    timestamp: float
    target_direction: float
    errors: Errors
    random_destination: plan_route.MapPosition


class State(TypedDict):
    """
    It represents the state that has to be carried between runs of the
    main loop.
    """
    data_directory: str
    button_A_was_on_last_cycle: bool
    button_A_toggle: bool
    destination: plan_route.MapPosition
    recording_state: RecordingState
    brand_new_destination: bool
    parsed_input: ParsedInput
    start_new_data_batch: bool
    write_data_to_disk: bool
    arrived: bool


class RawInput(TypedDict):
    """ It represents the raw input data, from various sources. """
    err_and_microbit: Tuple[str, bytes]
    err_and_gps: Tuple[str, bytes]
    err_and_colour_photo: Tuple[bool, 'np.ndarray[np.uint8]']
    timestamp: float
    err_and_target_direction: Tuple[str, float]
    random_destination: plan_route.MapPosition


def bytes2bool(b: bytes) -> bool:
    """ It converts a bytestring to a bool. """
    if b == b'True':
        return True
    if b == b'False':
        return False
    raise ValueError


def parse_microbit_reading(reading: bytes) -> Tuple[str, MicrobitReading]:
    """
    It converts the reading from the serial input, which is bytes, into
    a tuple of ints.
    """
    chunks = reading.split()
    print(chunks)
    if len(chunks) != 6:
        return "List does not have exactly six elements.", None
    try:
        return (
            None,
            MicrobitReading(
                compass=int(chunks[0]),
                accel=Acceleration(
                    x=int(chunks[1][1:-1]),
                    y=int(chunks[2][:-1]),
                    z=int(chunks[3][:-1])),
                buttonA=bytes2bool(chunks[4]),
                buttonB=bytes2bool(chunks[5])))
    except ValueError:
        return "Could not convert to int.", None


def angle_minutes_to_float(angle: str) -> float:
    """
    It converts the angle from degrees and minutes to just degrees.

    The input angle is a string like "5016.81116".  Everything after the
    decimal point are the decimals of the minutes of the angle, the two
    digits immediately to the left of the decimal point are the whole
    minutes, and everything to the left of that is the whole degrees.  One
    degree is 60 minutes.
    """
    first_half, minutes_decimals = angle.split('.')
    whole_minutes = first_half[-2:]
    degrees = first_half[:-2]
    return float(degrees) + float(whole_minutes + '.' + minutes_decimals) / 60


GPS_PARSER = parse.compile('$GPGLL,{:f},{:w},{:f},{:w},{:S}\r\n')


def parse_gps_reading(
        line_of_output: bytes) -> Tuple[str, plan_route.MapPosition]:
    """ It parses a line of the output from the GPS reader.  """
    as_string = line_of_output.decode('utf-8')
    parsed = GPS_PARSER.parse(as_string)
    if parsed is None:
        return "Could not parse GPS reading.", None
    if parsed[1] == 'N':
        latitude_sign = 1
    if parsed[1] == 'S':
        latitude_sign = -1
    if parsed[3] == 'E':
        longitude_sign = 1
    if parsed[3] == 'W':
        longitude_sign = -1
    return (None, plan_route.MapPosition(
        latitude=latitude_sign * angle_minutes_to_float(str(parsed[0])),
        longitude=longitude_sign * angle_minutes_to_float(str(parsed[2]))))


def write2file(
        parsed_input: ParsedInput,
        directory: str):
    """
    It writes the sensor readings to disk. The numeric data is converted
    to json format and appended to a file, and the photo is put in a folder
    called 'photos'.
    """
    with open(directory + '/sensor_readings.dat', 'a') as readings_file:
        json.dump(
            {'gps': parsed_input['gps'],
             'timestamp': parsed_input['timestamp'],
             'target_direction': parsed_input['target_direction']},
            readings_file)
    cv2.imwrite(
        '{}/photos/{}.jpg'.format(directory, parsed_input['timestamp']),
        parsed_input['grey_photo'])


def make_random_destination() -> plan_route.MapPosition:
    """ It generates a random location close to Kingsbridge, Devon, UK. """
    return plan_route.MapPosition(
        longitude=random.uniform(-3.7866, -3.7662),
        latitude=random.uniform(50.273, 50.293))


def is_close(a: plan_route.MapPosition, b: plan_route.MapPosition) -> bool:
    """
    It decides if two map positions are close, roughly within a few metres
    of each other.
    """
    return (
        (a.longitude - b.longitude)**2 + (a.latitude - b.latitude)**2
        < 0.00000001)


RECORDING_STATE_CODES = {
    RecordingState.ON: 1,
    RecordingState.OFF_PAUSED: 0,
    RecordingState.OFF_ARRIVED: 2}


class Output(TypedDict):
    """ The data to send to the outputs. """
    start_new_data_batch: bool
    data_directory: str
    write_data_to_disk: bool
    parsed_input: ParsedInput
    display: int


def state2output(state: State) -> Output:
    """
    It converts the state into the data needed for writing the output.
    """
    return {
        'start_new_data_batch': state['start_new_data_batch'],
        'data_directory': state['data_directory'],
        'write_data_to_disk': state['write_data_to_disk'],
        'parsed_input': state['parsed_input'],
        'display': calculate_view(
            state['parsed_input']['errors'],
            state['parsed_input']['target_direction'],
            state['parsed_input']['microbit']['compass'],
            state['recording_state'],
            state['brand_new_destination'])}


def calculate_view(
        errors: Errors,
        target_direction: float,
        actual_direction: float,
        recording_state: RecordingState,
        brand_new_destination: bool) -> int:
    """
    It creates the code that represents the display on the microbit LEDs.
    This is then sent to the microbit and decoded.  For the detail about
    what the codes mean, look at the comments in the docstring of the
    'display' function in the file 'microbit_code.py'.
    """
    if at_least_one_error(errors):
        error_code = 1
    else:
        error_code = 0

    recording_code = RECORDING_STATE_CODES[recording_state]

    direction_code = int(
        16 * (target_direction - actual_direction) / (2 * math.pi))

    if brand_new_destination:
        destination_just_created_code = 1
    else:
        destination_just_created_code = 0

    return (
        (error_code << 7) + (recording_code << 5) +
        (direction_code << 1) + destination_just_created_code)


def parse_input(raw_data: RawInput) -> ParsedInput:
    """ It parses the input data. """
    route_error, target_direction = raw_data['err_and_target_direction']
    gps_read_error, raw_gps_reading = raw_data['err_and_gps']
    if gps_read_error is None:
        gps_parse_error, gps_reading = parse_gps_reading(raw_gps_reading)
    else:
        gps_parse_error = None
        gps_reading = None
    gps_parse_error, gps_reading = parse_gps_reading(raw_gps_reading)
    photo_ok, colour_photo = raw_data['err_and_colour_photo']
    microbit_read_error, raw_microbit_readings = (
        raw_data['err_and_microbit'])
    if microbit_read_error is None:
        microbit_parse_error, microbit_readings = parse_microbit_reading(
            raw_microbit_readings)
    else:
        microbit_parse_error = None
        microbit_readings = None
    return {
        'microbit': microbit_readings,
        'gps': gps_reading,
        'grey_photo': cv2.cvtColor(colour_photo, cv2.COLOR_BGR2GRAY),
        'timestamp': raw_data['timestamp'],
        'target_direction': target_direction,
        'random_destination': raw_data['random_destination'],
        'errors': {
            'route': route_error,
            'gps_read': gps_read_error,
            'gps_parse': gps_parse_error,
            'photo_ok': photo_ok,
            'microbit_read': microbit_read_error,
            'microbit_parse': microbit_parse_error}}


def update_state(s: State, parsed_input: ParsedInput) -> State:
    """
    It calculates the new state, given the existing one, the sensor
    readings, a new random destination, and the target direction.
    """
    if at_least_one_error(parsed_input['errors']):
        s['parsed_input']['errors'] = parsed_input['errors']
        s['write_data_to_disk'] = False
        s['start_new_data_batch'] = False
        return s

    brand_new_destination = parsed_input['microbit']['buttonB']
    if brand_new_destination:
        destination = parsed_input['random_destination']
    else:
        destination = s['destination']

    # 1, 1 = 0
    # 1, 0 = 1
    # 0, 1 = 1
    # 0, 0 = 0
    button_A_toggle = (
        s['button_A_toggle'] != parsed_input['microbit']['buttonA'])

    arrived = is_close(parsed_input['gps'], destination)

    if s['button_A_toggle'] and parsed_input['microbit']['buttonA']:
        recording_state = RecordingState.OFF_PAUSED
    elif not s['arrived'] and arrived:  # i.e. just arrived
        recording_state = RecordingState.OFF_ARRIVED
    else:
        recording_state = RecordingState.ON

    start_new_data_batch = (
        parsed_input['microbit']['buttonA'] and not
        s['button_A_toggle'])
    if start_new_data_batch:
        data_directory = str(parsed_input['timestamp'])
    else:
        data_directory = s['data_directory']

    return {
        'data_directory': data_directory,
        'button_A_was_on_last_cycle': (
            s['parsed_input']['microbit']['buttonA']),
        'button_A_toggle': button_A_toggle,
        'destination': destination,
        'recording_state': recording_state,
        'brand_new_destination': brand_new_destination,
        'parsed_input': parsed_input,
        'start_new_data_batch': start_new_data_batch,
        'write_data_to_disk': recording_state == RecordingState.ON,
        'arrived': arrived}


def read_input(
        position: plan_route.MapPosition,
        destination: plan_route.MapPosition,
        webcam_handle,
        gps_serial_port,
        microbit_serial_port) -> RawInput:
    """
    It gathers data from the microbit, the gps receiver, the camera and
    the clock.
    """

    # Clear the buffers of the serial ports of the microbit and gps
    # reader.  This is so that the reading is fresh instead of being
    # the old data left in the buffer.
    gps_serial_port.reset_input_buffer()
    microbit_serial_port.reset_input_buffer()

    # The GPS receiver produces a batch of 7 lines of output for each
    # reading.  The final line of the seven is the one with the position
    # in it.  The for loop is for ignoring the first six lines.
    for _ in range(6):
        gps_serial_port.readline()

    return {
        'err_and_microbit': microbit_serial_port.readline(),
        'err_and_gps': gps_serial_port.readline(),
        'err_and_colour_photo': webcam_handle.read(),
        'timestamp': time.time(),
        'random_destination': make_random_destination(),
        'err_and_target_direction': plan_route.main(position, destination)}


def at_least_one_error(errors: Errors) -> bool:
    """ It decides if there is at least one error in the errors dict. """
    return (not all([v is None if k != 'photo_ok' else v
                     for k, v in errors]))


def send_output(output: Output, microbit_port):
    """ It writes out the output of various types. """
    if output['start_new_data_batch']:
        os.makedirs(output['data_directory'])
    if output['write_data_to_disk']:
        write2file(output['parsed_input'], output['data_directory'])
    microbit_port.write(output['display'])
    if at_least_one_error(output['parsed_input']['errors']):
        with open('error_log.txt', 'a') as error_file:
            json.dump(
                {'timestamp': output['parsed_input']['timestamp'],
                 'errors': output['parsed_input']['errors']},
                error_file)


def main():
    """
    It runs the main loop which runs continuously and does everything.
    """

    destination = make_random_destination()
    state: State = {
        'data_directory': 'realData/{}'.format(time.time()),
        'button_A_was_on_last_cycle': False,
        'button_A_toggle': False,
        'recording_state': RecordingState.OFF_PAUSED,
        'brand_new_destination': False,
        'destination': destination,
        'parsed_input': {
            'grey_photo': np.zeros(1, dypte=np.uint8),
            'microbit': {
                'accel': {'x': 0, 'y': 0, 'z': 0},
                'compass': 0,
                'buttonA': False,
                'buttonB': False},
            'gps': plan_route.MapPosition(latitude=0, longitude=0),
            'timestamp': 0,
            'target_direction': 0,
            'errors': {
                'route': None,
                'gps_read': None,
                'gps_parse': None,
                'photo_ok': True,
                'microbit_read': None,
                'microbit_parse': None},
            'random_destination': destination},
        'start_new_data_batch': True,
        'write_data_to_disk': False,
        'arrived': False}

    with cv2.VideoCapture(1) as webcam_handle, \
            serial.Serial('/dev/ttyACM0', baudrate=115200) as gps_port, \
            serial.Serial('/dev/ttyACM1', baudrate=115200) as microbit_port:
        while True:
            send_output(state2output(state), microbit_port)
            raw_input_data = read_input(
                state['position'],
                state['destination'],
                webcam_handle,
                gps_port,
                microbit_port)
            state = update_state(state, parse_input(raw_input_data))
