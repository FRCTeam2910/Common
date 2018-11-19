#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np

data = np.genfromtxt('trajectory.csv', delimiter=',', names=True)

plt.plot(data['time'], data['velocity'])
plt.plot(data['time'], data['maxVelocity'])
plt.show()
