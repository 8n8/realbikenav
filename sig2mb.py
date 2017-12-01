""" Temporary module to test sending a signal to the microbit. """

import serial  # type: ignore

port = serial.Serial('/dev/ttyACM1')
port.baudrate = 115200

port.write(b'123')
