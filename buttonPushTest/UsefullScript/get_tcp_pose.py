import rtde_control
import rtde_receive
from math import pi
import numpy as np
from scipy.spatial.transform import Rotation
import cv2

IP = "192.168.1.102"

def rotationToEuler(rotation_matrix):
    # Convert rotation matrix to Euler angles (XYZ convention)
    rotation = Rotation.from_matrix(rotation_matrix)
    euler_angles_xyz = rotation.as_euler('xyz', degrees=True)

    print("Euler Angles (XYZ):", euler_angles_xyz)
    
    return euler_angles_xyz

rtde_c = rtde_control.RTDEControlInterface(IP)
rtde_r = rtde_receive.RTDEReceiveInterface(IP)

tcp_pose = rtde_r.getActualTCPPose() 

print(tcp_pose)

joints = rtde_r.getActualQ()
print("joints", joints)

oriVec = np.array([-1.523036511755994, 0.4166201296350476, -0.41078296473354364])


rotation_matrix = cv2.Rodrigues(oriVec)

print(rotationToEuler(rotation_matrix[0]))
