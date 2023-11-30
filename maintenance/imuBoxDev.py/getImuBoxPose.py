import rtde_control
import rtde_receive
from math import pi
import numpy as np 
from scipy.spatial.transform import Rotation
from ArucoEstimationMatrix import findArucoLocation

IP = "192.168.1.102"

rtde_c = rtde_control.RTDEControlInterface(IP)
rtde_r = rtde_receive.RTDEReceiveInterface(IP)


# This script is used to make the robot end effector rotate relative to an aruco marker

def rotationToEuler(rotation_matrix):
    # Convert rotation matrix to Euler angles (XYZ convention)
    rotation = Rotation.from_matrix(rotation_matrix)
    euler_angles_xyz = rotation.as_euler('xyz', degrees=True)

    print("Euler Angles (XYZ):", euler_angles_xyz)
    
    return euler_angles_xyz[2]

def rad2deg(list):
    newlist = []
    for j in range(3):
        newlist.append(list[j])
    for i in range(3, 6):
        newlist.append(list[i] * (180/pi))

    return newlist

velocity = 0.08
acceleration = 0.08
blend = 0.0
rtde_c.setTcp([0, 0, 0.22, 0, 0, 0])
jointsImu = rtde_r.getActualQ()

print(jointsImu)

tcp_pose = rtde_r.getActualTCPPose() 

tcp_pose = rad2deg(tcp_pose)

print(tcp_pose)

framePose = [-0.01663474656286537, 0.2674566902637054, 0.4439993492629446, 
              np.radians(-85), np.radians(24), np.radians(-25)]

zero = [0,0,0,0,0,0]

#while True:
    #x_distance, y_distance, z_distance, ids, rotation_matrix = findArucoLocation()
    #if ids is not None and not isinstance(ids, str) and ids.any():
        #z_rot = rotationToEuler(rotation_matrix)
        #poseInFramePose = [0, 0, 0, 0, 0, np.radians(z_rot - 90)]
        #print(poseInFramePose)
        #break
#home = [0.34, 0.34, 0.285, np.deg2rad(-84), np.deg2rad(35), np.deg2rad(35)]

#pose_test = [0.34, 0.34, 0.285, np.radians(-84.92322114),  np.radians(35),  np.radians(-35)]

#inv = rtde_c.getInverseKinematics(pose_test)

pickPose = rtde_c.poseTrans(framePose, zero)
joints = rtde_r.getActualQ()
print(joints)

rtde_c.moveL(pickPose, velocity, acceleration, blend)
