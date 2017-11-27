"""
It constantly records the camera images and the corresponding compass
bearings.
"""

import json
import time
import os
import typing
import serial  # type: ignore
import cv2  # type: ignore
import parse  # type: ignore


class Acceleration(typing.NamedTuple):
    """ It represents the acceleration reading. """
    x: int
    y: int
    z: int


class Reading(typing.NamedTuple):
    """ It represents one reading of the microbit sensors. """
    accel: Acceleration
    compass: int
    buttonOn: bool


def bytes2bool(b: bytes) -> bool:
    """ It converts a bytestring to a bool. """
    if b == b'True':
        return True
    if b == b'False':
        return False
    raise ValueError


def parse_reading(reading: bytes) -> typing.Tuple[str, Reading]:
    """
    It converts the reading from the serial input, which is bytes, into
    a tuple of ints.
    """
    chunks = reading.split()
    print(chunks)
    if len(chunks) != 5:
        return "List does not have exactly five elements.", None
    try:
        return (
            None,
            Reading(
                compass=int(chunks[0]),
                accel=Acceleration(
                    x=int(chunks[1][1:-1]),
                    y=int(chunks[2][:-1]),
                    z=int(chunks[3][:-1])),
                buttonOn=bytes2bool(chunks[4])))
    except ValueError:
        return "Could not convert to int.", None


class Position(typing.NamedTuple):
    """ It records a map position. """
    latitude: float
    longitude: float


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


def parse_gps(line_of_output: bytes) -> typing.Tuple[str, Position]:
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
    return (None, Position(
        latitude=latitude_sign * angle_minutes_to_float(str(parsed[0])),
        longitude=longitude_sign * angle_minutes_to_float(str(parsed[2]))))


def main():
    """
    It runs the main loop which reads the sensors and decides what to do.
    """
    cap = cv2.VideoCapture(1)

    serial_port_microbit = serial.Serial('/dev/ttyACM0')
    serial_port_microbit.baudrate = 115200

    history = {}

    id_code = str(time.time())
    os.makedirs('realData/' + id_code + '/photos')
    os.makedirs('realData/' + id_code + '/sensor_readings')

    counter = 0
    while True:
        now = time.time()
        err, data = parse_reading(serial_port_microbit.readline())
        serial_port_microbit.reset_input_buffer()
        if err is not None:
            print(err)
            continue
        if not data.buttonOn:
            continue
        _, colour_photo = cap.read()
        grey_photo = cv2.cvtColor(colour_photo, cv2.COLOR_BGR2GRAY)
        cv2.imwrite(
            'realData/{}/photos/{}.jpg'.format(id_code, now),
            grey_photo)
        history[now] = data
        if counter != 0 and counter % 100 == 0:
            data_file_name = 'realData/{}/sensor_readings/{}.dat'.format(
                id_code,
                now)
            with open(data_file_name, 'w') as directions_file:
                json.dump(history, directions_file)
            history = {}
        counter += 1
