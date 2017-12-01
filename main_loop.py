"""
It constantly records the camera images and the corresponding microbit
readings, gps readings, and target direction from the routing algorithm.
"""

import json
import time
import os
from typing import NamedTuple, Tuple
import serial  # type: ignore
import cv2  # type: ignore
import numpy  # noqa: F401
import parse  # type: ignore
import plan_route
import random
import math
import enum


class Acceleration(NamedTuple):
    """ It represents the acceleration reading. """
    x: int
    y: int
    z: int


class MicrobitReading(NamedTuple):
    """ It represents one reading of the microbit sensors. """
    accel: Acceleration
    compass: int
    buttonA_just_pressed: bool
    buttonB_just_pressed: bool


class State(NamedTuple):
    data_directory: str
    button_A_was_on_last_cycle: bool
    button_A_toggle: bool
    destination: MapPosition
    recording_state: RecordingState
    brand_new_destination: bool
    actual_direction: float
    target_direction: float
    route_error: str
    sensor_error: str
    sensor_readings: SensorsReading
    start_new_data_batch: bool
    write_data_to_disk: bool
    arrived: bool


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
                buttonOn=bytes2bool(chunks[4]),
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


class SensorsReading(NamedTuple):
    """
    It contains one set of sensor readings, taken at one point in time.
    """
    gps: plan_route.MapPosition
    microbit: MicrobitReading
    target_bearing: float
    timestamp: float
    photo: 'numpy.ndarray[numpy.uint8]'


def write2file(s: SensorsReading, directory: str):
    """
    It writes the sensor readings to disk. The numeric data is converted
    to json format and appended to a file, and the photo is put in a folder
    called 'photos'.
    """
    with open(directory + '/sensor_readings.dat', 'a') as readings_file:
        json.dump(
            {'gps': s.gps,
             'microbit': s.microbit,
             'target_bearing': s.target_bearing,
             'timestamp': s.timestamp},
            readings_file)
    cv2.imwrite('{}/photos/{}.jpg'.format(directory, s.timestamp), s.photo)


def read_sensors(
        microbit_serial_port,
        gps_serial_port,
        webcam,
        destination: plan_route.MapPosition) -> Tuple[str, SensorsReading]:
    """ It reads the microbit, gps receiver and webcam. """

    # Take GPS reading.
    gps_serial_port.reset_input_buffer()
    # The GPS receiver produces a batch of 7 lines of output for each
    # reading.  The final line of the seven is the one with the position
    # in it.  The for loop is for ignoring the first six lines.
    for _ in range(6):
        gps_serial_port.readline()
    gps_error, position = parse_gps_reading(gps_serial_port.readline())
    if gps_error is not None:
        return 'Error with GPS reading: {}.'.format(gps_error), None

    # Take microbit reading.
    microbit_serial_port.reset_input_buffer()
    microbit_error, microbit_reading = parse_microbit_reading(
        microbit_serial_port.readline())
    if microbit_error is not None:
        return (
            'Error with microbit reading: {}.'.format(microbit_error),
            None)

    # Plan the route.
    map_error, target_bearing = plan_route.main(position, destination)
    if map_error is not None:
        return 'Map/routing error: {}'.format(map_error), None

    # Take photo.
    ok, colour_photo = webcam.read()
    if not ok:
        return 'There was an error taking the photo.', None
    black_and_white = cv2.cvtColor(colour_photo, cv2.COLOR_BGR2GRAY)

    return (
        None,
        SensorsReading(
            microbit=microbit_reading,
            photo=black_and_white,
            gps=position,
            target_bearing=target_bearing,
            timestamp=time.time()))


def random_destination() -> plan_route.MapPosition:
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


RECORDING_STATE_CODES = {
    RecordingState.ON: 1,
    RecordingState.OFF_PAUSED: 0,
    RecordingState.OFF_ARRIVED: 2}


def state2view(
        error: str,
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
    if error is None:
        error_code = 0
    else:
        error_code = 1

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


def state_updater(
        s: State,
        sensor_readings: SensorsReading,
        random_destination: plan_route.MapPosition,
        target_direction_with_err: Tuple[str, float],
        sensor_error) -> State:

    brand_new_destination = sensor_readings.microbit.buttonB_just_pressed
    if brand_new_destination:
        destination = random_destination
    else:
        destination = s.destination

    # 1, 1 = 0
    # 1, 0 = 1
    # 0, 1 = 1
    # 0, 0 = 0
    button_A_toggle = (
        s.button_A_toggle != sensor_readings.microbit.buttonA_just_pressed)
    
    arrived = isclose(sensor_readings.gps, destination)

    if s.button_A_toggle and sensor_readings.microbit.buttonA_just_pressed:
        recording_state = RecordingState.OFF_PAUSED
    elif not s.arrived and arrived:  # i.e. just arrived
        recording_state = RecordingState.OFF_ARRIVED
    else:
        recording_state = RecordingState.ON

    route_error, target_direction = target_direction_with_err
        
    start_new_data_batch = (
        sensor_readings.microbit.buttonA_just_pressed and not
        s.button_A_toggle)
    if start_new_data_batch:
        data_directory = new_data_dir
    else:
        data_directory = s.data_directory

    return State(
        data_directory=data_directory,
        button_A_was_on_last_cycle=
            s.sensor_readings.microbit.buttonA_just_pressed,
        button_A_toggle=button_A_toggle,
        destination=destination,
        recording_state=recording_state,
        brand_new_destination=brand_new_destination,
        actual_direction=(
            sensor_readings.microbit.compass * 2 * math.pi / 360.0),
        target_direction=target_direction,
        route_error=route_error,
        sensor_error=sensor_error,
        sensor_readings=sensor_readings,
        start_new_data_batch=start_new_data_batch,
        write_data_to_disk=(
            route_error is None and sensor_error is None and
            recording_state == RecordingState.ON)
        arrived=arrived)


def main(s: State, state_updater, state2view):
    """
    It runs the main loop which reads the sensors and decides what to do.
    """
    os.makedirs(s.data_directory)
    webcam_handle = cv2.VideoCapture(1)
    gps_serial_port = serial.Serial('/dev/ttyACM0', baudrate=115200)
    microbit_serial_port = serial.Serial('/dev/ttyACM1', baudrate=115200)
    os.makedirs(s.data_directory)

    while True:
        microbit_serial_port.write(state2view(s))
        sensor_error, sensor_readings = read_sensors(
            microbit_serial_port, gps_serial_port, webcam, s.destination)
        if start_new_data_batch:
            os.makedirs(s.data_directory)
        if write_data_to_disk:
            write2file(data, s.data_directory)
        s = state_updater(
            s,
            sensor_readings,
            random_destination(),
            plan_route.main(sensor_readings.gps, s.destination),
            sensor_error)

    cv2.ReleaseCapture(1)
