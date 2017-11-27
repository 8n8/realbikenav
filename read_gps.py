""" It reads the GPS receiver and prints out its output. """

import time
import serial  # type: ignore

serial_port = serial.Serial('/dev/ttyACM0')
serial_port.baudrate = 115200

while True:
    print(serial_port.readline())
    time.sleep(0.1)
