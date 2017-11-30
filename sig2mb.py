import serial

port = serial.Serial('/dev/ttyACM1')
port.baudrate = 115200

port.write(b'123')
