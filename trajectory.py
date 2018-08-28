#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np

data = np.genfromtxt('trajectory.csv', names=True, delimiter=',')

plt.plot(data['time'], data['rotation'])
plt.show()
