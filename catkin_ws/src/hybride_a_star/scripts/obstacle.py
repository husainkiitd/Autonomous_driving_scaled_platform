#!usr/bin/env python3
import cv2 
import numpy as np
import matplotlib.pyplot as plt
def obstacle_from_map():
    # Read the PGM file using OpenCV
    pgm_file = "my_map.pgm"
    #pgm_file = map_name
    img = cv2.imread(pgm_file, cv2.IMREAD_GRAYSCALE)

    # Set the threshold values
    occupied_thresh = 0.65
    free_thresh = 0.196

    # Convert the pixel values to binary values
    binary_img = np.zeros_like(img)
    binary_img[img >= occupied_thresh] = 1
    binary_img[img <= free_thresh] = 0

    # Set the origin and resolution
    resolution = 0.050000
    origin = [-51.224998, -51.224998]

    # Create the occupancy grid map
    map_width = binary_img.shape[1]
    map_height = binary_img.shape[0]
    occupancy_map = np.zeros((map_height, map_width))
    #obstacleList = []
    ox = []
    oy = []
    for i in range(map_height):
        for j in range(map_width):
            x = origin[0] + (j + 0.5) * resolution
            y = origin[1] + (map_height - i - 0.5) * resolution
            occupancy_map[i, j] = binary_img[i, j]
            if binary_img[i, j] == 0:
                ox.append(x)
                oy.append(y)
                #obstacleList.append((x, y, 0.2)) # assuming robot radius is 0.8 meters

    # Save the occupancy grid map as a text file
    #np.savetxt("occupancy_map.txt", occupancy_map, fmt="%d")

    # Visualize the occupancy grid map
    plt.imshow(occupancy_map, cmap="gray")
    plt.show()
    print(len(ox))
    return ox, oy


if __name__ == '__main__':

    ox, oy = obstacle_from_map()
        