#!/usr/bin/env python3
import numpy as np
from scipy.interpolate import CubicSpline

# Assuming you have an array of x and y coordinates from RRT
x_rrt = [1, 2, 3, 4, 5]
y_rrt = [2, 4, 5, 6, 8]

# Create a cubic spline interpolation
cs = CubicSpline(x_rrt, y_rrt)

# Generate new x and y coordinates for the smoothed path
x_smooth = np.linspace(x_rrt[0], x_rrt[-1], num=100)
y_smooth = cs(x_smooth)

print(x_smooth)
# Plot the smoothed path
import matplotlib.pyplot as plt
plt.plot(x_rrt, y_rrt, 'o', label='RRT Path')
plt.plot(x_smooth, y_smooth, label='Smoothed Path')
plt.legend()
plt.show()
