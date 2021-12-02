import cv2
import os
import numpy as np

""" This script serves only one purpose: expand the obstacles in a given
map, using parameters defined in a .world file for Stage. With this
expansion, the robot can be treated as a point. It is a mapping to the 
configuration space.
"""

parent_folder = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
map_path = os.path.join(parent_folder, 'worlds/maze.png') #Edit the path as needed
map_image = cv2.imread(map_path)
save_path = os.path.join(parent_folder, 'worlds/grid1.npy') #Edit the path as needed

# Stage Parameters, change these as needed
width, height = (16, 16) # floorpan size in Stage .world file (in meters)
scale = 28 # window scale paramter in Stage .world file (in meters / pixel)
robot_size = (1.2*np.sqrt(2), 1.2*np.sqrt(2)) # robot size defined in .world file (or any .inc)
initial = (-6, 2) # robot initial position defined in .world file

img_big = cv2.resize(map_image, (width * scale, height * scale), interpolation = cv2.INTER_AREA)
gray = cv2.cvtColor(img_big, cv2.COLOR_BGR2GRAY) # Grayscale image

# Tranposes the image, since opencv treats pictures as (height, width)
grid = cv2.transpose(gray.copy()) # (X x Y)
grid = (grid/255).astype(np.uint64) # binary grid
grid = 0**grid # sets 0->free space and 1->obstacles

# Expands the obstacles such that the robot can be considered a point
augmented_grid = grid.copy()
rx, ry = robot_size
rx = scale * rx
ry = scale * ry
xmax, ymax = grid.shape
for pixel in list(zip(*np.nonzero(grid == 1))):
    x, y = pixel
    augmented_grid[int(np.maximum(0, x - rx//2)) : int(np.minimum(xmax-1, x + rx//2)), 
                   int(np.maximum(0, y - ry//2)) : int(np.minimum(ymax-1, y + ry//2))] = 1

# Save expanded map as a matrix in .npy file
np.save(save_path, augmented_grid)