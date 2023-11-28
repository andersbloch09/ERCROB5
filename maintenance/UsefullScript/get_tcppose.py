import rtde_control
import rtde_receive
from math import pi
import numpy as np 

IP = "192.168.1.102"

rtde_c = rtde_control.RTDEControlInterface(IP)
rtde_r = rtde_receive.RTDEReceiveInterface(IP)


def rad2deg(list):
    newlist = []
    for j in range(3):
        newlist.append(list[j])
    for i in range(3, 6):
        newlist.append(list[i] * (180/pi))

    return newlist

tcp_pose = rtde_r.getActualTCPPose() 

tcp_pose = rad2deg(tcp_pose)

print(tcp_pose)

import numpy as np

def create_transformation_matrix(x, y, z, roll, pitch, yaw):
    """
    Create a 4x4 transformation matrix for a given position and orientation.
    """
    translation_matrix = np.array([
        [1, 0, 0, x],
        [0, 1, 0, y],
        [0, 0, 1, z],
        [0, 0, 0, 1]
    ])

    rotation_matrix = np.array([
        [np.cos(yaw)*np.cos(pitch), -np.sin(yaw)*np.cos(roll) + np.cos(yaw)*np.sin(pitch)*np.sin(roll), np.sin(yaw)*np.sin(roll) + np.cos(yaw)*np.sin(pitch)*np.cos(roll), 0],
        [np.sin(yaw)*np.cos(pitch), np.cos(yaw)*np.cos(roll) + np.sin(yaw)*np.sin(pitch)*np.sin(roll), -np.cos(yaw)*np.sin(roll) + np.sin(yaw)*np.sin(pitch)*np.cos(roll), 0],
        [-np.sin(pitch), np.cos(pitch)*np.sin(roll), np.cos(pitch)*np.cos(roll), 0],
        [0, 0, 0, 1]
    ])

    transformation_matrix = np.dot(translation_matrix, rotation_matrix)
    return transformation_matrix

# Example: Desired position and orientation
x = 0.19143747326267901
y = 0.20156817224782103
z = 0.43287980447795843
roll = np.radians(-85)  # Convert degrees to radians
pitch = np.radians(32)
yaw = np.radians(-36)

# Create the transformation matrix
transform_matrix = create_transformation_matrix(x, y, z, roll, pitch, yaw)

print("Transformation Matrix:")
print(transform_matrix)

p = np.array([0.1,0,0,1])

dot_product = np.dot(transform_matrix, p)

print(dot_product)