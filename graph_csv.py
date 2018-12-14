#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np

data = np.genfromtxt('trajectory.csv', delimiter=',', names=True)

plt.figure(1, figsize=(12, 8))

plt.subplot(231)
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Path')
plt.plot(data['x'], data['y'])

plt.subplot(232)
plt.xlabel('Time')
plt.ylabel('Feedforward')
plt.title('Feedforward')
plt.plot(data['time'], data['f'])

plt.subplot(234)
plt.xlabel('Time')
plt.ylabel('Position')
plt.title('Position')
plt.plot(data['time'], data['position'])

plt.subplot(235)
plt.xlabel('Time')
plt.ylabel('Velocity')
plt.title('Velocity')
plt.plot(data['time'], data['velocity'])

plt.subplot(236)
plt.xlabel('Time')
plt.ylabel('Acceleration')
plt.title('Acceleration')
plt.plot(data['time'], data['acceleration'])

plt.suptitle('Trajectory')
plt.show()
