""" It plots the compass readings from sensor_readings.dat. """

import json
import matplotlib.pyplot as plt  # type: ignore

with open('sensor_readings.dat', 'r') as data_file:
    data = [json.loads(line) for line in data_file.read().splitlines()]

compass = [d['microbit']['compass'] for d in data]
print([int(c) for c in compass])
print(len(compass))
print(type(compass[0]))
plt.plot(compass)
plt.show()
