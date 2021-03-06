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
from typing import Any, Dict, List, Tuple

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


class GpsReading(TypedDict):
    """ It represents the reading from the GPS receiver. """
    speed: float
    speed_error: str
    position: plan_route.MapPosition
    position_error: str


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
    gps_speed: str
    gps_position: str
    gps_buffer_too_small: str
    photo: str
    microbit: str
    random_destination: str


class ErrorCounters(TypedDict):
    """
    It contains the number of consecutive errors for each of the error
    types.
    """
    route: int
    gps_speed: int
    gps_position: int
    gps_buffer_too_small: int
    photo: int
    microbit: int
    random_destination: int


class ErrorResponse(TypedDict):
    """ It contains the decision about how to react to the errors. """
    log_messages: List[str]
    led_error: bool
    stop_program: bool


# The number of consecutive GPS errors that is acceptable before failing.
GPS_ERROR_TOLERANCE: int = 14


def error_handler(
        errors: Errors,
        error_counters: ErrorCounters) -> Tuple[ErrorCounters, ErrorResponse]:
    """
    It updates the error counts, makes the error log messages, decides
    whether to flash the error LEDs and decides whether to end the program.
    """
    log_messages = []
    led_error = False
    stop_program = False

    if errors['route'] is not None:
        error_counters['route'] += 1
        log_messages.append('route: ' + errors['route'])
        led_error = True
        if error_counters['route'] > 6:
            stop_program = True
    else:
        error_counters['route'] = 0

    if errors['gps_speed'] is not None:
        error_counters['gps_speed'] += 1
        if error_counters['gps_speed'] > GPS_ERROR_TOLERANCE:
            stop_program = True
            led_error = True
            log_messages.append('gps_speed: ' + errors['gps_speed'])
    else:
        error_counters['gps_speed'] = 0

    if errors['gps_position'] is not None:
        error_counters['gps_position'] += 1
        if error_counters['gps_position'] > GPS_ERROR_TOLERANCE:
            stop_program = True
            led_error = True
            log_messages.append(
                'gps_position: ' + errors['gps_position'])
    else:
        error_counters['gps_position'] = 0

    if errors['gps_buffer_too_small'] is not None:
        error_counters['gps_buffer_too_small'] += 1
        if error_counters['gps_buffer_too_small'] > GPS_ERROR_TOLERANCE:
            stop_program = True
            led_error = True
            log_messages.append(
                'gps_buffer_too_small: ' + errors['gps_buffer_too_small'])
    else:
        error_counters['gps_buffer_too_small'] = 0

    if errors['photo'] is not None:
        error_counters['photo'] += 1
        if error_counters['photo'] > 1:
            log_messages.append('photo: ' + errors['photo'])
        if error_counters['photo'] > 3:
            stop_program = True
            led_error = True
    else:
        error_counters['photo'] = 0

    if errors['microbit'] is not None:
        error_counters['microbit'] += 1
        if error_counters['microbit'] > 12:
            stop_program = True
            led_error = True
            log_messages.append('microbit: ' + errors['microbit'])
    else:
        error_counters['microbit'] = 0

    if errors['random_destination'] is not None:
        error_counters['random_destination'] += 1
        log_messages.append(
            'random_destination: ' + errors['random_destination'])
        if error_counters['random_destination'] > 10:
            stop_program = True
            led_error = True
    else:
        error_counters['random_destination'] = 0

    return (
        error_counters,
        {'log_messages': log_messages,
         'led_error': led_error,
         'stop_program': stop_program})


class ParsedInput(TypedDict):
    """ It contains the input data after parsing.  """
    microbit: MicrobitReading
    gps_position: plan_route.MapPosition
    gps_speed: float
    grey_photo: 'np.ndarray[np.uint8]'
    timestamp: float
    target_direction: float
    errors: Errors
    random_destination: plan_route.MapPosition


class KalmanState(TypedDict):
    """
    It contains the latest estimate of the state vector and the error
    covariance matrix.
    """
    # x contains the speed, directon, latitude and longitude
    x: 'np.ndarray[np.float64]'
    P: 'np.ndarray[np.float64]'


LENGTH_OF_DEGREE_LATITUDE: float = 111000  # metres


def kalman_filter(
        state: KalmanState,
        gps_speed: float,
        compass: float,
        timestep: float,
        position: plan_route.MapPosition,
        gps_position: plan_route.MapPosition) -> KalmanState:
    """ It updates the state using a Kalman filter. """
    # x contains the speed, direction, latitude and longitude
    length_of_degree_longitude: float = (
        math.cos(position.latitude) * 111000)  # metres
    xk_1k_1 = state['x']
    Pk_1k_1 = state['P']
    # Predicted new latitude is:
    #  old_lat + cos(direction) * speed * degrees_lat_per_metre * timestep
    lat_speed_multiplier: float = (
        timestep * math.cos(xk_1k_1[1]) /  # type: ignore
        LENGTH_OF_DEGREE_LATITUDE)
    # Predicted new longitude is:
    #  old_lon + sin(direction) * speed * degrees_lon_per_metre * timestep
    lon_speed_multiplier: float = (
        timestep * math.sin(xk_1k_1[1]) /  # type: ignore
        length_of_degree_longitude)
    Fk = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [lat_speed_multiplier, 0, 1, 0],
        [lon_speed_multiplier, 0, 0, 1]])
    xkk_1 = np.matmul(Fk, xk_1k_1)

    if gps_speed is None:
        speed: float = xkk_1[0][0]  # type: ignore
    else:
        speed = gps_speed

    if compass is None:
        direction: float = xkk_1[1][0]  # type: ignore
    else:
        direction = compass

    if gps_position is None:
        latitude: float = xkk_1[2][0]  # type: ignore
        longitude: float = xkk_1[3][0]  # type: ignore
    else:
        latitude = gps_position.latitude
        longitude = gps_position.longitude

    zk = np.array([[speed], [direction], [latitude], [longitude]])
    identity = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]])
    Qk = identity
    Hk = identity
    Rk = identity
    Pkk_1 = np.matmul(Fk, np.matmul(Pk_1k_1, Fk.T)) + Qk
    Sk = Rk + np.matmul(  # type: ignore
        Hk, np.matmul(Pkk_1, np.linalg.inv(Hk)))  # type: ignore
    Kk = np.matmul(Pkk_1, np.matmul(Hk.T, np.linalg.inv(Sk)))  # type: ignore
    yk = zk - np.matmul(Hk, xkk_1)
    return {
        'x': xkk_1 + np.matmul(Kk, yk),  # type: ignore
        'P': Pkk_1 - np.matmul(Kk, np.matmul(Hk, Pkk_1))}  # type: ignore


class State(TypedDict):
    """
    It represents the state that has to be carried between runs of the
    main loop.
    """
    counter: int
    data_directory: str
    direction: float
    speed: float
    time_of_last_update: float
    position: plan_route.MapPosition
    button_A_toggle: bool
    destination: plan_route.MapPosition
    recording_state: RecordingState
    brand_new_destination: bool
    parsed_input: ParsedInput
    start_new_data_batch: bool
    write_data_to_disk: bool
    arrived: bool
    error_response: ErrorResponse
    error_counters: ErrorCounters
    kalman_state: KalmanState


class RawInput(TypedDict):
    """ It represents the raw input data, from various sources. """
    err_and_microbit: Tuple[str, MicrobitReading]
    err_and_gps: Tuple[str, GpsReading]
    err_and_colour_photo: Tuple[bool, 'np.ndarray[np.uint8]']
    timestamp: float
    err_and_target_direction: Tuple[str, float]
    err_and_random_destination: Tuple[str, plan_route.MapPosition]


def parse_microbit_reading(reading: bytes) -> Tuple[str, MicrobitReading]:
    """
    It converts the reading from the serial input, which is bytes, into
    a tuple of ints.
    """
    # cutoff = 40
    # if len(reading) > cutoff:
    #     reading = reading[-cutoff:]
    as_string = reading.decode('utf-8')
    chunks = as_string.split()
    if len(chunks) != 6:
        return "Byte array does not have exactly 24 elements.", None
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


GPS_POSITION_PARSER = parse.compile('$GPGLL,{:f},{:w},{:f},{:w},{:S}\r\n')


def parse_gps_position_reading(
        line_of_output: bytes) -> Tuple[str, plan_route.MapPosition]:
    """
    It parses a line of the output from the GPS reader that contains the
    position.
    """
    as_string = line_of_output.decode('utf-8')
    parsed = GPS_POSITION_PARSER.parse(as_string)
    if parsed is None:
        return "Could not parse GPS position reading.", None
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


GPS_SPEED_PARSER = parse.compile('$GPVTG,{},N,{:f},K{}')


def parse_gps_speed_reading(line_of_output: bytes) -> Tuple[str, float]:
    """
    It parses the line of the output from the GPS reader that contains the
    speed.
    """
    as_string = line_of_output.decode('utf-8')
    parsed = GPS_SPEED_PARSER.parse(as_string)
    if parsed is None:
        return "Could not parse GPS speed reading.", None
    # The raw reading is in km/hour, so to convert it into metres / seconde
    # it is multiplied by 1000 / (60*60) = 0.277777778.
    return None, parsed[-2] * 0.277777778


def write2file(
        parsed_input: ParsedInput,
        destination: plan_route.MapPosition,
        speed: float,
        directory: str):
    """
    It writes the sensor readings to disk. The numeric data is converted
    to json format and appended to a file, and the photo is put in a folder
    called 'photos'.
    """
    json4file = json.dumps({'microbit': parsed_input['microbit'],
         'destination': destination,
         'speed': speed,
         'gps_speed_reading': parsed_input['gps_speed'],
         'gps_position': parsed_input['gps_position'],
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


def make_random_destination() -> Tuple[str, plan_route.MapPosition]:
    """ It generates a random location close to Kingsbridge, Devon, UK. """
    return plan_route.nearest_point(plan_route.MapPosition(
        longitude=random.uniform(-3.7866, -3.7662),
        latitude=random.uniform(50.273, 50.293)))


def is_close(a: plan_route.MapPosition, b: plan_route.MapPosition) -> bool:
    """
    It decides if two map positions are close, roughly within a few metres
    of each other.
    """
    return plan_route.distance_between(a, b) < 20


RECORDING_STATE_CODES = {
    RecordingState.ON: 1,
    RecordingState.OFF_PAUSED: 0,
    RecordingState.OFF_ARRIVED: 2}


class Output(TypedDict):
    """ The data to send to the outputs. """
    counter: int
    start_new_data_batch: bool
    speed: float
    data_directory: str
    write_data_to_disk: bool
    parsed_input: ParsedInput
    destination: plan_route.MapPosition
    display: int
    error_response: ErrorResponse


def state2output(state: State) -> Output:
    """
    It converts the state into the data needed for writing the output.
    """
    return {
        'counter': state['counter'],
        'start_new_data_batch': state['start_new_data_batch'],
        'data_directory': state['data_directory'],
        'write_data_to_disk': state['write_data_to_disk'],
        'parsed_input': state['parsed_input'],
        'destination': state['destination'],
        'speed': state['speed'],
        'error_response': state['error_response'],
        'display': calculate_view(
            state['error_response']['led_error'],
            state['parsed_input']['target_direction'],
            state['direction'],
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
        led_error: bool,
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
    error_code = int(led_error)

    recording_code = RECORDING_STATE_CODES[recording_state]

    direction_code: int = make_direction_code(
        target_direction, actual_direction)

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

    gps_err, gps_reading = raw_data['err_and_gps']

    photo_ok, colour_photo = raw_data['err_and_colour_photo']
    if photo_ok:
        photo_error: str = None
    else:
        photo_error = "Bad photo."

    microbit_error, microbit_reading = raw_data['err_and_microbit']

    destination_err, random_destination = (
        raw_data['err_and_random_destination'])

    return {
        'microbit': microbit_reading,
        'gps_position': gps_reading['position'],
        'gps_speed': gps_reading['speed'],
        'grey_photo': cv2.cvtColor(colour_photo, cv2.COLOR_BGR2GRAY),
        'timestamp': raw_data['timestamp'],
        'target_direction': target_direction,
        'random_destination': random_destination,
        'errors': {
            'route': route_error,
            'gps_speed': gps_reading['speed_error'],
            'gps_position': gps_reading['position_error'],
            'gps_buffer_too_small': gps_err,
            'photo': photo_error,
            'random_destination': destination_err,
            'microbit': microbit_error}}


# Used as a case statement to choose the right recording state from the
# two bools 'just_arrived' and 'buttonAtoggle'.
#
# (just_arrived, buttonAtoggle): RecordingState
RECORDING_SWITCH: Dict[Tuple[bool, bool], RecordingState] = {
    (False, False): RecordingState.OFF_PAUSED,
    (False, True): RecordingState.ON,
    (True, False): RecordingState.OFF_ARRIVED,
    (True, True): RecordingState.OFF_ARRIVED}


def update_state(s: State, parsed_input: ParsedInput) -> State:
    """
    It calculates the new state, given the existing one, the sensor
    readings, a new random destination, and the target direction.
    """

    error_counters, error_response = error_handler(
        parsed_input['errors'],
        s['error_counters'])

    kalman_state = kalman_filter(
        s['kalman_state'],
        parsed_input['gps_speed'],
        parsed_input['microbit']['compass'],
        parsed_input['timestamp'] - s['time_of_last_update'],
        s['position'],
        parsed_input['gps_position'])

    speed: float = kalman_state['x'][0][0]  # type: ignore

    direction: float = kalman_state['x'][1][0]  # type: ignore
    print('direction is {}'.format(direction))

    position: plan_route.MapPosition = plan_route.MapPosition(  # type: ignore
        latitude=kalman_state['x'][2][0],  # type: ignore
        longitude=kalman_state['x'][3][0])  # type: ignore

    buttonApressed: bool = (
        s['parsed_input']['microbit']['buttonAcounter'] !=
        parsed_input['microbit']['buttonAcounter'])

    # Toggle changing, with (oldToggle, buttonPressed) = newToggle
    # 1, 1 = 0
    # 1, 0 = 1
    # 0, 0 = 0
    # 0, 1 = 1
    # This is equivalent to oldToggle != buttonPressed

    buttonAtoggle: bool = s['button_A_toggle'] != buttonApressed

    buttonBpressed: bool = (
        s['parsed_input']['microbit']['buttonBcounter'] !=
        parsed_input['microbit']['buttonBcounter'])

    brand_new_destination: bool = buttonBpressed
    if brand_new_destination:
        destination: plan_route.MapPosition = parsed_input[
            'random_destination']
    else:
        destination = s['destination']

    arrived: bool = is_close(position, destination)

    just_arrived: bool = not s['arrived'] and arrived

    recording_state: RecordingState = RECORDING_SWITCH[
        (just_arrived, buttonAtoggle)]

    # So start a new data batch if buttonAtoggle has just been switched on.
    start_new_data_batch = (not s['button_A_toggle']) and buttonAtoggle
    if start_new_data_batch:
        data_directory = 'realData/' + str(parsed_input['timestamp'])
    else:
        data_directory = s['data_directory']

    return {
        'counter': s['counter'] + 1,
        'kalman_state': kalman_state,
        'position': position,
        'error_response': error_response,
        'error_counters': error_counters,
        'direction': direction,
        'speed': speed,
        'data_directory': data_directory,
        'button_A_toggle': buttonAtoggle,
        'destination': destination,
        'recording_state': recording_state,
        'time_of_last_update': parsed_input['timestamp'],
        'brand_new_destination': brand_new_destination,
        'parsed_input': parsed_input,
        'start_new_data_batch': start_new_data_batch,
        'write_data_to_disk': recording_state == RecordingState.ON,
        'arrived': arrived}


def read_gps(gps_serial_port) -> Tuple[str, GpsReading]:
    """
    It tries several times to read the GPS receiver and get a valid
    reading.
    """
    reading: GpsReading = {
        'speed': None,
        'speed_error': 'No speed reading available.',
        'position': None,
        'position_error': 'No position reading available.'}
    waiting = gps_serial_port.inWaiting()
    if waiting < 500:
        return 'Buffer not full enough.', reading
    for _ in range(14):
        raw = gps_serial_port.readline()
        if reading['speed_error'] is not None:
            reading['speed_error'], reading['speed'] = (
                parse_gps_speed_reading(raw))
        if reading['position_error'] is not None:
            reading['position_error'], reading['position'] = (
                parse_gps_position_reading(raw))
        if (reading['speed_error'] is None and
                reading['position_error'] is None):
            break
    return None, reading


def read_microbit(microbit_serial_port) -> Tuple[str, MicrobitReading]:
    """
    It tries several times to read the microbit and get a valid reading.
    """
    err = 'could not get valid microbit reading'
    for _ in range(3):
        raw = microbit_serial_port.readline()
        err, parsed = parse_microbit_reading(raw)
        if err is None:
            return None, parsed
    return err, None


def read_input(
        position: plan_route.MapPosition,
        destination: plan_route.MapPosition,
        brand_new_destination: bool,
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
    # microbit_serial_port.reset_input_buffer()
    if brand_new_destination:
        random_destination = make_random_destination()
    else:
        random_destination = None, destination
    return {
        'err_and_microbit': read_microbit(microbit_serial_port),
        'err_and_gps': read_gps(gps_serial_port),
        'err_and_colour_photo': webcam_handle.read(),
        'timestamp': time.time(),
        'err_and_random_destination': random_destination,
        'err_and_target_direction': plan_route.main(position, destination)}


def empty(L: List[Any]) -> bool:
    """
    It returns true if the list is empty.  I do it like this because I can
    never remember whether [] is True or False.
    """
    return not L


def send_output(output: Output, microbit_port):
    """ It sends out the output to the file system and the microbit. """
    print(output['counter'])
    if output['start_new_data_batch']:
        os.makedirs(output['data_directory'] + '/photos')
    if output['write_data_to_disk']:
        write2file(output['parsed_input'], output['destination'],
                   output['speed'], output['data_directory'])
    microbit_port.write([output['display']])
    if not empty(output['error_response']['log_messages']):
        print(output['error_response']['log_messages'])
        with open('error_log.txt', 'a') as error_file:
            error_file.write(json.dumps(
                {'timestamp': output['parsed_input']['timestamp'],
                 'errors': output['error_response']['log_messages']}) + '\n')


def initial_state() -> State:
    """ It creates the initial state. """
    destination_error, destination = make_random_destination()
    position = plan_route.MapPosition(latitude=50.289658,
                                      longitude=-3.7740055)
    return {
        'counter': 0,
        'data_directory': 'realData/{}'.format(time.time()),
        'error_response': {
            'log_messages': [],
            'led_error': False,
            'stop_program': False},
        'direction': 0,
        'button_A_toggle': False,
        'speed': 0,
        'time_of_last_update': time.time(),
        'position': position,
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
            'gps_position': position,
            'gps_speed': 0,
            'timestamp': 0,
            'target_direction': 0,
            'errors': {
                'route': None,
                'gps_position': None,
                'gps_speed': None,
                'gps_buffer_too_small': None,
                'photo': None,
                'random_destination': destination_error,
                'microbit': None},
            'random_destination': destination},
        'start_new_data_batch': False,
        'error_counters': {
            'route': 0,
            'gps_position': 0,
            'gps_speed': 0,
            'gps_buffer_too_small': 0,
            'photo': 0,
            'random_destination': 0,
            'microbit': 0},
        'write_data_to_disk': False,
        'kalman_state': {
            'x': np.zeros((4, 1)),
            'P': np.zeros((4, 4))},
        'arrived': False}


def main():
    """
    It runs the main loop which runs continuously and does everything.
    """
    state: State = initial_state()
    webcam_handle = cv2.VideoCapture(1)
    with serial.Serial('/dev/ttyACM0', baudrate=115200) as gps_port, \
            serial.Serial('/dev/ttyACM1', baudrate=115200) as microbit_port:
        while not state['error_response']['stop_program']:
            send_output(state2output(state), microbit_port)
            raw_input_data = read_input(
                state['position'],
                state['destination'],
                state['brand_new_destination'],
                webcam_handle,
                gps_port,
                microbit_port)
            state = update_state(state, parse_input(raw_input_data))


main()
