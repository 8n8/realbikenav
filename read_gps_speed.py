""" It reads the GPS receiver and prints out its output. """

import serial  # type: ignore

serial_port = serial.Serial('/dev/ttyACM0')
serial_port.baudrate = 115200

while True:
    reading = serial_port.readline()
    for line in reading.splitlines():
        print(line)
        if line[5:8] == 'VTG':
            print(line)
