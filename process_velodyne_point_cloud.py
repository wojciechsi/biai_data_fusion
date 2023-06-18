"""Module for processing point cloud data to get an occupancy grid"""

import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt

# Define grid parameters
VOXEL_SIZE = 0.1
GRID_THERESHOLD = 2 # minimal number of points in cell
RANGE = 255

# Define working area
SCANNER_HEIGHT = 1.73
MAX_HEIGHT_OF_OBSTACLE = 2.0
MIN_HEIGHT_OF_VALID_POINT = 0.2 # todo:     take in consideration value change
                                #           due to car pitch and roll
MAX_DISTANCE_FORWARD = 20.0
MAX_DISTANCE_SIDEWAYS = 10.0

def find_cell_index(scanned_point, voxel_size,
                    max_distance_sideways):
    voxel_x = int(scanned_point[0] / voxel_size)
    voxel_y = int((scanned_point[1] + max_distance_sideways) / voxel_size)
    return (voxel_x, voxel_y)

def create_occupancy_grid(filename):
    # Load binary point cloud
    bin_pcd = np.fromfile(filename, dtype=np.float32)

    # Reshape and drop reflection values
    points = bin_pcd.reshape((-1, 4))[:, 0:3]

    # Convert to Open3D point cloud
    point_cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))

    # Create a voxel grid
    voxel_grid = np.zeros((int(MAX_DISTANCE_FORWARD / VOXEL_SIZE), int(MAX_DISTANCE_SIDEWAYS * 2 / VOXEL_SIZE)))
    outout_cloud = []

    # Iterate through each point in the point cloud
    for point in point_cloud.points:
        # Calibrate the height of the point
        point[2] += SCANNER_HEIGHT
        # Truncate obstacles above a thereshold
        if point[2] > MAX_HEIGHT_OF_OBSTACLE:
            continue
        if point[2] < MIN_HEIGHT_OF_VALID_POINT:
            continue
        # Truncate obstacles that are too far away
        if point[0] > MAX_DISTANCE_FORWARD:
            continue
        if point[0] < 0:
            continue
        if abs(point[1]) > MAX_DISTANCE_SIDEWAYS:
            continue
        # Process the valid point
        cell_index = find_cell_index(point, VOXEL_SIZE, MAX_DISTANCE_SIDEWAYS)
        voxel_grid[cell_index] += 1
        outout_cloud.append(point)
    # Apply thereshold
    voxel_grid[voxel_grid < GRID_THERESHOLD] = 0

    # Normalize grid
    voxel_grid /= RANGE

    return voxel_grid, outout_cloud

### Main ###

grid, cloud = create_occupancy_grid("testing/velodyne/000000.bin")

def dzianis(anything):
    pass

dzianis(cloud)

# Visualize grid
plt.imshow(grid)
plt.show()

# Visualize point cloud with open3d
#o3d.visualization.draw_geometries([point_cloud])