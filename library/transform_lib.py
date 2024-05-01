import numpy as np

import torch

def convert_coordinates(coordinates):
    converted_coordinates = [(coord[0].detach().cpu(), coord[1].detach().cpu()) for coord in coordinates]
    return converted_coordinates

# Transformation matrix
transformation_matrix = np.array([[9.69256665e-06, -5.75229752e-03, 1.63064755e+00],
                                  [-5.97151264e-03, -3.37375011e-08, 1.05791849e+00],
                                  [-6.62936062e-07, 4.47307772e-04, 1.00000000e+00]])


def transform_coordinates(camera_coordinates):
    
    transformed_coords = []
   
    camera_coordinates = convert_coordinates(camera_coordinates)
   
    for coord in camera_coordinates:
        # Create homogeneous coordinates [x, y, 1]
        homogeneous_coords = np.array([[coord[0]],
                                       [coord[1]],
                                       [1]])

        # Apply transformation matrix
        real_world_coords = np.dot(transformation_matrix, homogeneous_coords)

        # Normalize the homogeneous coordinates
        real_world_coords /= real_world_coords[2]

        # Extract x and y coordinates from the transformed coordinates
        real_world_x = round(real_world_coords[0][0], 3)
        real_world_y = round(real_world_coords[1][0], 3)

        transformed_coords.append([real_world_x, real_world_y])

    return transformed_coords
