import numpy as np 
from . import gripperControl
from . import ArucoEstimationSmall as ArucoEstS
from . import ArucoEstimation as ArucoEst
from scipy.spatial.transform import Rotation
from . import CameraOffset as co
from math import pi
import time

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

def goToImuTable(rtde_c):
    velocity = 1
    acceleration = 1
    blend_1 = 0.0
    
    homeJoints = [2.0313563346862793, -1.352398232822754, -1.2709603309631348, -2.1171041927733363, 1.5746040344238281, -4.980692211781637]

    rtde_c.moveJ(homeJoints, velocity, acceleration, blend_1)


def moveToBoxOrientation(z_rot, boxHomeGlobalRef, rtde_c, boxFitLoc, rtde_r):
    velocity = 1
    acceleration = 1
    blend = 0
    # Orientate

    zOri = [0, 0, 0, 0, 0, np.radians(z_rot)]
    
    imuOrientation = rtde_c.poseTrans(boxHomeGlobalRef, zOri)

    rtde_c.moveL(imuOrientation, velocity, acceleration, blend)


    while len(boxFitLoc) < 1:
        x_distance, y_distance, z_distance, ids, rotation_matrix = ArucoEstS.findArucoLocation()
        if ids is not None and not isinstance(ids, str) and ids.any():
            boxFitLoc = co.imuBoxLocationFit(x_distance, y_distance, z_distance)

    boxPos = [boxFitLoc[0], boxFitLoc[1], 0, 0, 0, np.radians(90)]

    tcp_pose = rtde_r.getActualTCPPose() 

    #tcp_pose = rad2deg(tcp_pose)

    print(tcp_pose)

    zero = [0,0,0,0,0,0]

    boxPosRef = rtde_c.poseTrans(tcp_pose, boxPos)

    rtde_c.moveL(boxPosRef, velocity, acceleration, blend)

    return z_distance 


def boxPickUp(gripperImuBox, rtde_r, rtde_c, z_distance):
    velocity = 1
    acceleration = 1
    blend = 0
    
    tcp_pose = rtde_r.getActualTCPPose() 

    zDistToBox = co.imuBoxLocationPickup(z_distance)

    boxDepth = [0,0,zDistToBox,0,0,0]

    boxPosRef = rtde_c.poseTrans(tcp_pose, boxDepth)

    rtde_c.moveL(boxPosRef, velocity, acceleration, blend)

    gripperControl.gripperControl(gripperImuBox)
    
    boxDepth = [0,0,0,0,0,0]

    boxPosRef = rtde_c.poseTrans(tcp_pose, boxDepth)

    rtde_c.moveL(boxPosRef, velocity, acceleration, blend)


def findImuBox(rtde_c, rtde_r, gripperImuBox):
    boxFitLoc = []
    boxHomeGlobalRef = [0.0463559417774228, 0.24843588058249952, 0.17441032897587316, 
              np.radians(-165.44384144), np.radians(68.5), np.radians(-0.59804425)]
    z_rot = None
    while z_rot is None:
        x_distance, y_distance, z_distance, ids, rotation_matrix = ArucoEstS.findArucoLocation()
        if ids is not None and not isinstance(ids, str) and ids.any():
            z_rot = rotationToEuler(rotation_matrix)

    z_distance = moveToBoxOrientation(z_rot, boxHomeGlobalRef, rtde_c, boxFitLoc, rtde_r)
    boxPickUp(gripperImuBox, rtde_r, rtde_c, z_distance)

def scanImuBoardLoc(rtde_c, rtde_r): 
    velocity = 0.9
    acceleration = 0.9
    blend_1 = 0.0
    
    boardScan = [2.127626895904541, -1.622178693810934, -1.3549747467041016, 3.5395304399677734, -0.5716679731952112, -9.864086278269085]

    rtde_c.moveJ(boardScan, velocity, acceleration, blend_1)

    boardPose = None

    while boardPose is None:
        x_distance, y_distance, z_distance, ids = ArucoEst.findArucoLocation()
        if ids is not None and not isinstance(ids, str) and ids.any():
            boardPose = co.arucoToBoardImu(x_distance, y_distance, z_distance)


    boardPoseRef = rtde_r.getActualTCPPose() 


    # these angles should be changes
    boardPoseRef = [boardPoseRef[0], boardPoseRef[1], boardPoseRef[2], np.deg2rad(-86.7), np.deg2rad(23.2), np.deg2rad(-23.6)]

    boardPose = [boardPose[0], boardPose[1], boardPose[2], 0, 0, 0]

    return boardPoseRef, boardPose

def placeImu(imuAngle, boardPoseRef, boardPose, rtde_c, rtde_r, gripperOpen):
    velocity = 0.2
    acceleration = 0.2
    blend = 0.0

    boardScan = rtde_c.getInverseKinematics(boardPoseRef)
    
    #boardScan = [2.127626895904541, -1.622178693810934, -1.3549747467041016, 3.5395304399677734, -0.5716679731952112, -9.864086278269085]

    rtde_c.moveJ(boardScan, velocity, acceleration)

    velocity = 0.5
    acceleration = 0.5
    blend = 0.0

    boardPose[2] = boardPose[2] - 0.07
    boardPose[5] = imuAngle

    boxPosRef = rtde_c.poseTrans(boardPoseRef, boardPose)

    rtde_c.moveL(boxPosRef, velocity, acceleration, blend)

    boardPose[2] = boardPose[2] + 0.08

    boxPosRef = rtde_c.poseTrans(boardPoseRef, boardPose)

    rtde_c.moveL(boxPosRef, velocity, acceleration, blend)
    
    gripperControl.gripperControl(gripperOpen)
    
    time.sleep(1)
    
    boardPose[2] = boardPose[2] - 0.07

    boxPosRef = rtde_c.poseTrans(boardPoseRef, boardPose)

    rtde_c.moveL(boxPosRef, velocity, acceleration, blend)