import numpy as np 
from . import gripperControl
from . import ArucoEstimationSmall as ArucoEstS
from . import ArucoEstimation as ArucoEst
from . import CameraOffset as co
from math import pi
import time

def scanTable(rtde_c, rtde_r):
    velocity = 1
    acceleration = 1
    blend_1 = 0.0

    tableHomeJ = [0.8427166938781738, -2.0175072155394496, -0.7320303916931152, 1.1957790094562988, -1.5637462774859827, -9.366041310617717]
    rtde_c.moveJ(tableHomeJ, velocity, acceleration, blend_1)
    
    tableFitLoc = []

    while len(tableFitLoc) < 1:
        x_distance, y_distance, z_distance, ids = ArucoEst.findArucoLocation()
        if ids is not None and not isinstance(ids, str) and ids.any():
            tableFitLoc = co.lidTableLoc(x_distance, y_distance, z_distance)
    
    return tableFitLoc

def lidLocation(rtde_c, rtde_r): 
    velocity = 1
    acceleration = 1 

    # Tcp orientation of secret box scan
    #roll, pitch, yaw = -79.11510627, 48.48179948, -49.33543757

    #scanBoxPose = [0.44369501072561013, 0.09707740459758275, 0.15540484236620697, np.deg2rad(roll), np.deg2rad(pitch), np.deg2rad(yaw)]
    
    scanBoxJoints = [0.600801944732666, -1.433079556827881, -2.5380635261535645,
                      3.9887096124836425, -1.446258846913473, -9.430879720041546]

    rtde_c.moveJ(scanBoxJoints, velocity, acceleration)

    boxLoc = []

    while len(boxLoc) < 1:
        x_distance, y_distance, z_distance, ids = ArucoEst.findArucoLocation()
        if ids is not None and not isinstance(ids, str) and ids.any():
            boxLoc = co.lidBoxLoc(x_distance, y_distance, z_distance)
    
    return boxLoc

def pickUpLid(rtde_c, rtde_r, boxLoc, gripperSecretLid, tableFitLoc, gripperOpen):
    velocity = 1
    acceleration = 1
    blend = 0.0
    rtde_c.setTcp([0, 0, 0.19, 0, 0, 0])

    # Tcp orientation of secret box scan
    roll, pitch, yaw = -79.11510627, 48.48179948, -49.33543757

    scanBoxPose = [0.44369501072561013, 0.09707740459758275, 0.15540484236620697,
                    np.deg2rad(roll), np.deg2rad(pitch), np.deg2rad(yaw)]
    
    avoidBoxOne = [0, -0.1, -0.05, 0, 0, 0]

    boxPosRef = rtde_c.poseTrans(scanBoxPose, avoidBoxOne)

    rtde_c.moveL(boxPosRef, velocity, acceleration, blend)

    # Here the joints are saved for the returnment of the lid
    returnJoints = rtde_r.getActualQ()

    avoidBoxOne = [boxLoc[0], boxLoc[1], boxLoc[2], np.radians(-30), 0, 0]

    boxPosRefReturn = rtde_c.poseTrans(scanBoxPose, avoidBoxOne)

    rtde_c.moveL(boxPosRefReturn, velocity, acceleration, blend)

    gripperControl.gripperControl(gripperSecretLid)

    avoidBoxOne = [boxLoc[0], boxLoc[1] - 0.01, boxLoc[2], np.radians(-30), 0, 0]

    boxPosRef = rtde_c.poseTrans(scanBoxPose, avoidBoxOne)

    rtde_c.moveL(boxPosRef, velocity, acceleration, blend)

    avoidBoxOne = [0, boxLoc[1] - 0.01, -0.05, np.radians(-30), 0, 0]

    boxPosRef = rtde_c.poseTrans(scanBoxPose, avoidBoxOne)

    rtde_c.moveL(boxPosRef, velocity, acceleration, blend)

    avoidBoxOne = [0, boxLoc[1] + 0.09, -0.05, np.radians(-30), 0, 0]

    boxPosRef = rtde_c.poseTrans(scanBoxPose, avoidBoxOne)

    rtde_c.moveL(boxPosRef, velocity, acceleration, blend)

    scanTableBoxJoints = [0.8427166938781738, -2.0175072155394496, -0.7320303916931152, 
                     1.1957790094562988, -1.5637462774859827, -9.366041310617717]

    rtde_c.moveJ(scanTableBoxJoints, velocity, acceleration)

    # Lay the lid down 
    tableFitLoc = [tableFitLoc[0],tableFitLoc[1], tableFitLoc[2] - 0.01, np.radians(40), 0, 0]

    tcp_pose = rtde_r.getActualTCPPose() 

    tableFitLoc = rtde_c.poseTrans(tcp_pose, tableFitLoc)

    jointsTableFit = rtde_c.getInverseKinematics(tableFitLoc)
    
    rtde_c.moveJ(jointsTableFit, velocity, acceleration)

    gripperControl.gripperControl(gripperOpen)

    leaveLid = [0, 0, -0.08, 0, 0, 0]

    tcp_pose = rtde_r.getActualTCPPose() 

    leaveLid = rtde_c.poseTrans(tcp_pose, leaveLid)

    leaveLid = rtde_c.getInverseKinematics(leaveLid)

    rtde_c.moveJ(leaveLid, velocity, acceleration, blend)

    return leaveLid, returnJoints, boxPosRefReturn

def scanSecretAruco(rtde_c, rtde_r):
    velocity = 0.5
    acceleration = 0.5
    scanJoints = [0.4701828956604004, -1.7107840977110804, -0.6793193817138672,
                   4.63053481161084, 1.5090956687927246, -12.595390748961854]

    rtde_c.moveJ(scanJoints, velocity, acceleration)

    secretId = None

    while secretId is None:
        x_distance, y_distance, z_distance, ids = ArucoEst.findArucoLocation()
        if ids is not None and not isinstance(ids, str) and ids.any():
            secretId = ids[0][0]
    print("secret ID", secretId)
    return secretId

def returnLid(rtde_c, rtde_r, leaveLid, gripperSecretLid, returnJoints, boxPosRefReturn, gripperOpen):
    velocity = 1
    acceleration = 1
    blend = 0.0

    rtde_c.moveJ(leaveLid, velocity, acceleration)

    tableLidPos = [0, 0, 0.08, 0, 0, 0]

    tcp_pose = rtde_r.getActualTCPPose() 

    leaveLid = rtde_c.poseTrans(tcp_pose, tableLidPos)

    rtde_c.moveL(leaveLid, velocity, acceleration, blend)

    gripperControl.gripperControl(gripperSecretLid)

    tableLidPosAbove = [0, 0, -0.05, 0, 0, 0]

    tableLidPosAbove = rtde_c.poseTrans(tcp_pose, tableLidPosAbove)

    rtde_c.moveL(tableLidPosAbove, velocity, acceleration, blend)

    rtde_c.moveJ(returnJoints, velocity, acceleration)
    
    rtde_c.moveL(boxPosRefReturn, velocity, acceleration, blend)

    gripperControl.gripperControl(gripperOpen)

    rtde_c.moveJ(returnJoints, velocity, acceleration)




