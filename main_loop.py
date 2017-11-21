"""
It constantly records the camera images and the corresponding compass
bearings.
"""

import json
import time
import uuid
import os
import typing
import serial  # type: ignore
import cv2  # type: ignore

cap = cv2.VideoCapture(0)

serial_port = serial.Serial('/dev/ttyACM0')
serial_port.baudrate = 115200

history = {}

id_code = str(time.time())
os.makedirs('realData/' + id_code + '/photos')
os.makedirs('realData/' + id_code + '/sensor_readings')

compass_bearing = None

counter = 0

NUMS = [i + 49 for i in range(10)]


class Acceleration(typing.NamedTuple):
    """ It represents the acceleration reading. """
    x: int
    y: int
    z: int


class Reading(typing.NamedTuple):
    """ It represents one reading of the microbit sensors. """
    accel: Acceleration
    compass: int


def parse_reading(reading: bytes) -> typing.Tuple[str, Reading]:
    """
    It converts the reading from the serial input, which is bytes, into
    a tuple of ints.
    """
    chunks = reading.split()
    if len(chunks) != 4:
        return "List does not have exactly three elements.", None
    try:
        return (
            None,
            Reading(
                compass=int(chunks[0]),
                accel=Acceleration(
                    x=int(chunks[1][1:-1]),
                    y=int(chunks[2][:-1]),
                    z=int(chunks[3][:-1]))))
    except ValueError:
        return "Could not convert to int.", None


while True:
    now = time.time()
    err, data = parse_reading(serial_port.readline())
    if err is not None:
        print(err)
        continue
    _, colour_photo = cap.read()
    grey_photo = cv2.cvtColor(colour_photo, cv2.COLOR_BGR2GRAY)
    cv2.imwrite(
        'realData/{}/photos/{}.jpg'.format(id_code, now),
        grey_photo)
    history[now] = data
    if counter % 100 == 0:
        data_file_name = 'realData/{}/sensor_readings/{}.dat'.format(
            id_code,
            now)
        with open(data_file_name, 'w') as directions_file:
            json.dump(history, directions_file)
        history = {}
    counter += 1
