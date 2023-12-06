import numpy as np
from . import gripperControl
from . import ArucoEstimationSmall as ArucoEstS
from . import ArucoEstimation as ArucoEst
from scipy.spatial.transform import Rotation
from . import CameraOffset as co
from math import pi
import time


# Function is used to find the euler rotation of the z axis of the box
def rotationToEuler(rotation_matrix):
    # Convert rotation matrix to Euler angles (XYZ convention)
    rotation = Rotation.from_matrix(rotation_matrix)
    euler_angles_xyz = rotation.as_euler('xyz', degrees=True)

    print("Euler Angles (XYZ):", euler_angles_xyz)

    return euler_angles_xyz[2]


# This function takes a list of 6 and change the last 3
# values to radians for cartesian points
def rad2deg(list):
    newlist = []
    for j in range(3):
        newlist.append(list[j])
    for i in range(3, 6):
        newlist.append(list[i] * (180/pi))

    return newlist


# Goes to imuTalbe based on joints
def goToImuTable(rtde_c):
    velocity = 1
    acceleration = 1
    blend_1 = 0.0

    homeJoints = [1.883070468902588, -1.5635655683330079, -0.9728736877441406,
                  -2.205181737939352, 1.5705437660217285, -5.128586594258444]
    rtde_c.moveJ(homeJoints, velocity, acceleration, blend_1)


# Firstly moves the z axis of the robot and then scans again
# It does this to fit the orientation
# A transformation matrix should be used to scan only once
def moveToBoxOrientation(z_rot, boxHomeGlobalRef, rtde_c, boxFitLoc, rtde_r):
    velocity = 1
    acceleration = 1
    blend = 0

    zOri = [0, 0, 0, 0, 0, np.radians(z_rot)]

    imuOrientation = rtde_c.poseTrans(boxHomeGlobalRef, zOri)

    rtde_c.moveL(imuOrientation, velocity, acceleration, blend)

    while len(boxFitLoc) < 1:
        x_distance, y_distance, z_distance, \
            ids, rotation_matrix = ArucoEstS.findArucoLocation()
        # Makes sure the object is not a string so .any() can be used
        if ids is not None and not isinstance(ids, str) and ids.any():
            boxFitLoc = co.imuBoxLocationFit(
                x_distance, y_distance, z_distance)

    boxPos = [boxFitLoc[0], boxFitLoc[1], 0,
              0, 0, np.radians(90)]

    tcp_pose = rtde_r.getActualTCPPose()

    boxPosRef = rtde_c.poseTrans(tcp_pose, boxPos)

    rtde_c.moveL(boxPosRef, velocity, acceleration, blend)

    return z_distance


# This function uses the current TCP position
# to make a frame for picking up the box
def boxPickUp(gripperImuBox, rtde_r, rtde_c, z_distance):
    velocity = 1
    acceleration = 1
    blend = 0

    tcp_pose = rtde_r.getActualTCPPose()

    zDistToBox = co.imuBoxLocationPickup(z_distance)

    boxDepth = [0, 0, zDistToBox, 0, 0, 0]

    boxPosRef = rtde_c.poseTrans(tcp_pose, boxDepth)

    rtde_c.moveL(boxPosRef, velocity, acceleration, blend)

    gripperControl.gripperControl(gripperImuBox)

    boxDepth = [0, 0, 0, 0, 0, 0]

    boxPosRef = rtde_c.poseTrans(tcp_pose, boxDepth)

    rtde_c.moveL(boxPosRef, velocity, acceleration, blend)


# The function finds the box and then finds the rotation first
# afterwards it moves to the box oritation and runs the boxPickUp function
def findImuBox(rtde_c, rtde_r, gripperImuBox):
    boxFitLoc = []
    boxHomeGlobalRef = [0.04569544322714206, 0.277373367733082,
                        0.20183367305691477, np.radians(-165.44384144),
                        np.radians(68.5), np.radians(-0.59804425)]
    z_rot = None
    while z_rot is None:
        x_distance, y_distance, z_distance, \
            ids, rotation_matrix = ArucoEstS.findArucoLocation()
        if ids is not None and not isinstance(ids, str) and ids.any():
            z_rot = rotationToEuler(rotation_matrix)

    z_distance = moveToBoxOrientation(
        z_rot, boxHomeGlobalRef, rtde_c, boxFitLoc, rtde_r)

    boxPickUp(gripperImuBox, rtde_r, rtde_c, z_distance)


# Function moves to the imuBoard to find the location of where to put the imu
# then it returns it and the reference pose to is to create the frame
def scanImuBoardLoc(rtde_c, rtde_r):
    velocity = 0.9
    acceleration = 0.9
    blend_1 = 0.0

    boardScan = [2.127626895904541, -1.622178693810934, -1.3549747467041016,
                 3.5395304399677734, -0.5716679731952112, -9.864086278269085]

    rtde_c.moveJ(boardScan, velocity, acceleration, blend_1)

    boardPose = None

    while boardPose is None:
        x_distance, y_distance, z_distance, ids = ArucoEst.findArucoLocation()
        if ids is not None and not isinstance(ids, str) and ids.any():
            boardPose = co.arucoToBoardImu(x_distance, y_distance, z_distance)

    boardPoseRef = rtde_r.getActualTCPPose()

    roll, pitch, yaw = -86.7, 23.2, -23.6

    # these angles should be changes

    boardPoseRef = [boardPoseRef[0], boardPoseRef[1], boardPoseRef[2],
                    np.deg2rad(roll), np.deg2rad(pitch), np.deg2rad(yaw)]

    # The 0.07 is to keep a wanted distance
    # to the board so the box does not hit it

    boardPose = [boardPose[0], boardPose[1], boardPose[2] - 0.07, 0, 0, 0]

    return boardPoseRef, boardPose


# This function places the imu on  the board
# by using the inversekinematics solution
def placeImu(boardPoseRef, boardPose, rtde_c, rtde_r, gripperOpen, imuAngle):
    velocity = 0.2
    acceleration = 0.2
    blend = 0.0

    boardPoseTrans = rtde_c.poseTrans(boardPoseRef, boardPose)

    boardScan = rtde_c.getInverseKinematics(boardPoseTrans)

    rtde_c.moveJ(boardScan, velocity, acceleration)
    # boardScan = [2.127626895904541, -1.622178693810934, -1.3549747467041016,
    # 3.5395304399677734, -0.5716679731952112, -9.864086278269085]

    boardPose[5] = np.radians(imuAngle)

    boxPosRef = rtde_c.poseTrans(boardPoseRef, boardPose)

    rtde_c.moveL(boxPosRef, velocity, acceleration, blend)

    # The 0.0085 is the distance to reach the board due to
    # the aruco being measured directly and the board is not even with the plane
    boardPose[2] = boardPose[2] + 0.085

    boxPosRef = rtde_c.poseTrans(boardPoseRef, boardPose)

    rtde_c.moveL(boxPosRef, velocity, acceleration, blend)

    gripperControl.gripperControl(gripperOpen)

    time.sleep(1)

    boardPose[2] = boardPose[2] - 0.07

    boxPosRef = rtde_c.poseTrans(boardPoseRef, boardPose)

    rtde_c.moveL(boxPosRef, velocity, acceleration, blend)
