import numpy as np 
from . import gripperControl
from . import ArucoEstimationSmall as ArucoEstS
from scipy.spatial.transform import Rotation
from . import CameraOffset as co

def rotationToEuler(rotation_matrix):
    # Convert rotation matrix to Euler angles (XYZ convention)
    rotation = Rotation.from_matrix(rotation_matrix)
    euler_angles_xyz = rotation.as_euler('xyz', degrees=True)

    print("Euler Angles (XYZ):", euler_angles_xyz)
    
    return euler_angles_xyz[2]

def goToImuTable(rtde_c, gripperOpen):
    velocity = 1
    acceleration = 1
    blend_1 = 0.0
    
    gripperControl.gripperControl(gripperOpen)
    homeJoints = [2.0313563346862793, -1.352398232822754, -1.2709603309631348, -2.1171041927733363, 1.5746040344238281, -4.980692211781637]

    rtde_c.moveJ(homeJoints, velocity, acceleration, blend_1)

def findImuBox():
    imuBoxLoc = []
    while not imuBoxLoc:
        x_distance, y_distance, z_distance, ids, rotation_matrix = ArucoEstS.findArucoLocation()
        if ids is not None and not isinstance(ids, str) and ids.any():
            z_rot = rotationToEuler(rotation_matrix)
            co.imuBoxLocation(x_distance, y_distance, z_distance, z_rot)
            # Problem In this function have a hard time seeing the aruco probably due to light
