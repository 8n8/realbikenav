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
from typing import Dict, Tuple

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
    compass: float
    buttonAcounter: int
    buttonBcounter: int


class Errors(TypedDict):
    """
    It contains all the error messages arising from reading the sensors and
    calculating the route.
    """
    route: str
    gps: str
    photo_ok: bool
    microbit: str


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
    gps_error_count: int
    data_directory: str
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
    err_and_microbit: Tuple[str, MicrobitReading]
    err_and_gps: Tuple[str, plan_route.MapPosition]
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
    # cutoff = 40
    # if len(reading) > cutoff:
    #     reading = reading[-cutoff:]
    chunks = reading.split()
    if len(chunks) != 6:
        print(chunks)
        return "List does not have exactly six elements.", None
    try:
        return (
            None,
            MicrobitReading(
                compass=float(chunks[0])*2*math.pi/360,
                accel=Acceleration(
                    x=int(chunks[1]),
                    y=int(chunks[2]),
                    z=int(chunks[3])),
                buttonAcounter=int(chunks[4]),
                buttonBcounter=int(chunks[5])))
    except ValueError:
        print(chunks)
        return "Could not convert to numeric.", None


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
    json4file = json.dumps(
        {'microbit': parsed_input['microbit'],
         'gps': parsed_input['gps'],
         'timestamp': parsed_input['timestamp'],
         'target_direction': parsed_input['target_direction']})
    with open('{}/sensor_readings.dat'
              ''.format(directory), 'a') as readings_file:
        readings_file.write(json4file + '\n')

    cv2.imwrite(
        '{}/photos/{}.jpg'.format(
            directory,
            parsed_input['timestamp']),
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
            state['gps_error_count'],
            state['parsed_input']['errors'],
            state['parsed_input']['target_direction'],
            state['parsed_input']['microbit']['compass'],
            state['recording_state'],
            state['brand_new_destination'])}


def make_direction_code(
        target_direction: float,
        actual_direction: float) -> int:
    """
    It calculates an int between 0 and 15 to mark the direction on the
    microbit display.
    """
    diff = target_direction - actual_direction
    if diff < 0:
        diff = diff + 2*math.pi

    return int(16 * diff / (2 * math.pi))


def calculate_view(
        gps_error_count: int,
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
    error_code = int(errors['route'] is not None or
                     not errors['photo_ok'] or
                     errors['microbit'] is not None or
                     gps_error_count > 20)

    recording_code = RECORDING_STATE_CODES[recording_state]

    direction_code: int = make_direction_code(
        target_direction, actual_direction)

    if brand_new_destination:
        destination_just_created_code = 1
    else:
        destination_just_created_code = 0

    result = (
        (error_code << 7) + (recording_code << 5) +
        (direction_code << 1) + destination_just_created_code)
    if result < 0:
        print('error_code is {}'.format(error_code))
        print('recording_code is {}'.format(recording_code))
        print('direction_code is {}'.format(direction_code))
        print('destination_just_created_code is {}'.format(
            destination_just_created_code))
    return result


def parse_input(raw_data: RawInput) -> ParsedInput:
    """ It parses the input data. """

    route_error, target_direction = raw_data['err_and_target_direction']

    gps_error, gps_reading = raw_data['err_and_gps']

    photo_ok, colour_photo = raw_data['err_and_colour_photo']

    microbit_error, microbit_reading = raw_data['err_and_microbit']

    return {
        'microbit': microbit_reading,
        'gps': gps_reading,
        'grey_photo': cv2.cvtColor(colour_photo, cv2.COLOR_BGR2GRAY),
        'timestamp': raw_data['timestamp'],
        'target_direction': target_direction,
        'random_destination': raw_data['random_destination'],
        'errors': {
            'route': route_error,
            'gps': gps_error,
            'photo_ok': photo_ok,
            'microbit': microbit_error}}


def update_state(s: State, parsed_input: ParsedInput) -> State:
    """
    It calculates the new state, given the existing one, the sensor
    readings, a new random destination, and the target direction.
    """
    gps_error = parsed_input['errors']['gps'] is not None
    if gps_error:
        gps_error_count = s['gps_error_count'] + 1
        parsed_input['gps'] = s['parsed_input']['gps']
    else:
        gps_error_count = 0

    if (parsed_input['errors']['route'] is not None or
            not parsed_input['errors']['photo_ok'] or
            parsed_input['errors']['microbit'] is not None):
        s['parsed_input']['errors'] = parsed_input['errors']
        s['write_data_to_disk'] = False
        s['start_new_data_batch'] = False
        s['parsed_input']['timestamp'] = parsed_input['timestamp']
        return s

    # Toggle changing, with (oldToggle, buttonPressed) = newToggle
    # 1, 1 = 0
    # 1, 0 = 1
    # 0, 0 = 0
    # 0, 1 = 1

    buttonApressed = (
        s['parsed_input']['microbit']['buttonAcounter'] !=
        parsed_input['microbit']['buttonAcounter'])

    buttonAtoggle = s['button_A_toggle'] != buttonApressed

    buttonBpressed = (
        s['parsed_input']['microbit']['buttonBcounter'] !=
        parsed_input['microbit']['buttonBcounter'])

    brand_new_destination = buttonBpressed
    if brand_new_destination:
        destination = parsed_input['random_destination']
    else:
        destination = s['destination']

    arrived = is_close(parsed_input['gps'], destination)

    just_arrived = not s['arrived'] and arrived

    # (just_arrived, buttonAtoggle): RecordingState
    recording_switch: Dict[Tuple[bool, bool], RecordingState] = {
        (False, False): RecordingState.OFF_PAUSED,
        (False, True): RecordingState.ON,
        (True, False): RecordingState.OFF_ARRIVED,
        (True, True): RecordingState.OFF_ARRIVED}
    recording_state = recording_switch[(just_arrived, buttonAtoggle)]

    start_new_data_batch = (not s['button_A_toggle']) and buttonAtoggle
    if start_new_data_batch:
        data_directory = 'realData/' + str(parsed_input['timestamp'])
    else:
        data_directory = s['data_directory']

    return {
        'gps_error_count': gps_error_count,
        'data_directory': data_directory,
        'button_A_toggle': buttonAtoggle,
        'destination': destination,
        'recording_state': recording_state,
        'brand_new_destination': brand_new_destination,
        'parsed_input': parsed_input,
        'start_new_data_batch': start_new_data_batch,
        'write_data_to_disk': recording_state == RecordingState.ON,
        'arrived': arrived}


def read_gps(gps_serial_port) -> Tuple[str, plan_route.MapPosition]:
    """
    It tries several times to read the GPS receiver and get a valid
    reading.
    """
    err = 'not None'
    if gps_serial_port.inWaiting() < 500:
        return 'Buffer not full enough.', None
    for _ in range(14):
        raw = gps_serial_port.readline()
        err, parsed = parse_gps_reading(raw)
        if err is None:
            return None, parsed
    return err, None


def read_microbit(microbit_serial_port) -> Tuple[str, MicrobitReading]:
    """
    It tries several times to read the microbit and get a valid reading.
    """
    err = 'not None'
    for _ in range(3):
        raw = microbit_serial_port.readline()
        err, parsed = parse_microbit_reading(raw)
        if err is None:
            return None, parsed
    return err, None


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
    # gps_serial_port.reset_input_buffer()
    microbit_serial_port.reset_input_buffer()

    destination = make_random_destination()
    mb = read_microbit(microbit_serial_port)
    err, _ = mb
    return {
        'err_and_microbit': mb,
        'err_and_gps': read_gps(gps_serial_port),
        'err_and_colour_photo': webcam_handle.read(),
        'timestamp': time.time(),
        'random_destination': destination,
        'err_and_target_direction': plan_route.main(position, destination)}


def at_least_one_error(errors: Errors) -> bool:
    """ It decides if there is at least one error in the errors dict. """
    return (not all([v is None if k != 'photo_ok' else v
                     for k, v in errors.items()]))


def send_output(output: Output, microbit_port):
    """ It writes out the output of various types. """
    if output['start_new_data_batch']:
        os.makedirs(output['data_directory'] + '/photos')
    if output['write_data_to_disk']:
        write2file(output['parsed_input'], output['data_directory'])
    print('display is {}'.format(output['display']))
    microbit_port.write([output['display']])
    errors = output['parsed_input']['errors']
    # if at_least_one_error(output['parsed_input']['errors']):
    if (errors['route'] is not None or not errors['photo_ok']
            or errors['microbit'] is not None):
        print(output['parsed_input']['errors'])
        with open('error_log.txt', 'a') as error_file:
            error_file.write(json.dumps(
                {'timestamp': output['parsed_input']['timestamp'],
                 'errors': output['parsed_input']['errors']}) + '\n')


def main():
    """
    It runs the main loop which runs continuously and does everything.
    """

    destination = make_random_destination()
    state: State = {
        'gps_error_count': 0,
        'data_directory': 'realData/{}'.format(time.time()),
        'button_A_toggle': False,
        'button_B_toggle': False,
        'recording_state': RecordingState.OFF_PAUSED,
        'brand_new_destination': False,
        'destination': destination,
        'parsed_input': {
            'grey_photo': np.zeros(1),
            'microbit': {
                'accel': {'x': 0, 'y': 0, 'z': 0},
                'compass': 0.0,
                'buttonAcounter': 0,
                'buttonBcounter': 0},
            'gps': plan_route.MapPosition(
                latitude=50.289658, longitude=-3.7740055),
            'timestamp': 0,
            'target_direction': 0,
            'errors': {
                'route': None,
                'gps': None,
                'photo_ok': True,
                'microbit': None},
            'random_destination': destination},
        'start_new_data_batch': False,
        'write_data_to_disk': False,
        'arrived': False}

    webcam_handle = cv2.VideoCapture(0)

    with serial.Serial('/dev/ttyACM1', baudrate=115200) as gps_port, \
            serial.Serial('/dev/ttyACM0', baudrate=115200) as microbit_port:
        counter = 0
        while True:
            print('counter is {}'.format(counter))
            send_output(state2output(state), microbit_port)
            raw_input_data = read_input(
                state['parsed_input']['gps'],
                state['destination'],
                webcam_handle,
                gps_port,
                microbit_port)
            state = update_state(state, parse_input(raw_input_data))
            counter += 1


main()
