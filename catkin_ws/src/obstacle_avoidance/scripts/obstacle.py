#!usr/bin/env python3

"""
import numpy as np

def detect_obstacle(polar_ranges):
    # Find the indices of ranges within 0 to 1 meter
    indices = np.where(np.logical_and(polar_ranges[:, 0] > 0, polar_ranges[:, 0] < 1))[0]
    # Find the minimum distance within the y-range of -0.5 to +0.5
    min_dist = np.inf
    for i in indices:
        if abs(polar_ranges[i, 1]) < 0.5:
            min_dist = min(min_dist, polar_ranges[i, 0])
    # Return True if an obstacle is detected, False otherwise
    return min_dist < np.inf

if __name__ == '__main__':
    # Example usage:
    polar_ranges = np.array([[1.5, 0.1], [4, 1], [4, 2], [4, -2], [1.1, 0.6]])
    obstacle_detected = detect_obstacle(polar_ranges)
    print("Obstacle detected:", obstacle_detected)

"""

import numpy as np

def detect_obstacle(polar_ranges):
    # Find ranges with angles within the specified range
    x_indices = np.where((polar_ranges[:, 1] >= -0.5) & (polar_ranges[:, 1] <= 0.5))[0]
    y_indices = np.where((polar_ranges[:, 0] >= 0) & (polar_ranges[:, 0] <= 1))[0]
    obstacle_indices = np.intersect1d(x_indices, y_indices)
    # Check if there are any obstacles
    if len(obstacle_indices) > 0:
        obstacle_ranges = polar_ranges[obstacle_indices, :]
        # Find the closest obstacle
        closest_obstacle_idx = np.argmin(obstacle_ranges[:, 0])
        closest_obstacle_range = obstacle_ranges[closest_obstacle_idx, 0]
        closest_obstacle_angle = obstacle_ranges[closest_obstacle_idx, 1]
        print(f"Obstacle detected at range {closest_obstacle_range:.2f} meters and angle {np.rad2deg(closest_obstacle_angle):.2f} degrees.")
    else:
        print("No obstacle detected.")

if __name__ == '__main__':
    # Example usage:
    polar_ranges = np.array([[1.5, 0.1], [4, 1], [4, 2], [4, -2], [1.1, 0.6]])
    detect_obstacle(polar_ranges)
