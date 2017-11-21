""" To test reading the compass direction from the microbit. """

import time
import serial  # type: ignore

port = '/dev/ttyACM0'
baud = 115200
s = serial.Serial(port)
s.baudrate = baud

while True:
    data = s.readline()
    print(data)
    time.sleep(0.1)
