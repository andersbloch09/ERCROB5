import numpy as np
from scipy.spatial.transform import Rotation


def rotationToEuler(rotation_matrix):
    # Assuming you have a rotation matrix
    rotation_matrix = np.array([[-0.562713  , -0.82612145,  0.02962155],
        [-0.79344472,  0.54981556,  0.26105234],
        [-0.23194733,  0.12339448, -0.96487006]])

    # Convert rotation matrix to Euler angles (XYZ convention)
    rotation = Rotation.from_matrix(rotation_matrix)
    euler_angles_xyz = rotation.as_euler('xyz', degrees=True)

    print("Euler Angles (XYZ):", euler_angles_xyz)
