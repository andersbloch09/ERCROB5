import rtde_control
import rtde_receive
from math import pi
import numpy as np
from mainFolder.CameraOffset import buttonLocation
from mainFolder.ArucoEstimation import findArucoLocation
from mainFolder.gripperControl import gripperControl
from mainFolder.imuBoxMovement import goToImuTable, findImuBox, scanImuBoardLoc, placeImu
from mainFolder.secretBoxMovement import scanTable, lidLocation, pickUpLid, scanSecretAruco, returnLid
import time

IP = "192.168.1.102"

rtde_c = rtde_control.RTDEControlInterface(IP)
rtde_r = rtde_receive.RTDEReceiveInterface(IP)

gripperOpen = "open"
gripperClosed = "close"
gripperImuBox = "imu"
gripperSecretLid = "secretLid"

buttonList = []

buttonString = "123456789"

imuAngle = 45

class buttonObject():
    def __init__(self, id, loc, boardNumber):
        self.id = id
        self.loc = loc
        self.boardNumber = boardNumber


def getGridLength(pose1, velocity, acceleration, blend):
    gridButtons = []
    y = -0.04
    x = -0.02
    zeroPose = [0, 0, 0, 0, 0, 0]
    # Add center button to gridButtons for reference
    x_dist, y_dist, z_dist, ids = findArucoLocation()
    buttonPos = buttonLocation(zeroPose, x_dist, y_dist, z_dist)
    centerRef = buttonObject(ids[0][0], buttonPos, 4)

    gridButtons.append(centerRef)

    while len(gridButtons) < 2:
        x_dist, y_dist, z_dist, ids = findArucoLocation()
        # Check for found buttons
        if ids is not None and not isinstance(ids, str) and ids.any():
            if gridButtons[0].id != ids[0][0]:
                buttonPos = buttonLocation(pose2, x_dist, y_dist, z_dist)
                topRef = buttonObject(ids[0][0], buttonPos, 4)

                gridButtons.append(topRef)
                break

        y -= 0.04
        pose2 = [0, y, 0, 0, 0, 0]
        poseFound = rtde_c.poseTrans(pose1, pose2)

        poseFound.extend([velocity, acceleration, blend])

        path = [poseFound]
        
        rtde_c.moveL(path)
    
    while len(gridButtons) < 3:
        x_dist, y_dist, z_dist, ids = findArucoLocation()
        
        # Check for found buttons
        if ids is not None and not isinstance(ids, str) and ids.any():
            if gridButtons[1].id != ids[0][0]:
                buttonPos = buttonLocation(pose2, x_dist, y_dist, z_dist)
                leftRef = buttonObject(ids[0][0], buttonPos, 4)

                gridButtons.append(leftRef)
                break
        
            
        x -= 0.02
        pose2 = [x, y, 0, 0, 0, 0]
        poseFound = rtde_c.poseTrans(pose1, pose2)

        poseFound.extend([velocity, acceleration, blend])

        path = [poseFound]

        rtde_c.moveL(path)
        rtde_c.stopL(3, True)

    yLenght = abs(gridButtons[0].loc[1]) + abs(gridButtons[1].loc[1])
    xLenght = abs(gridButtons[1].loc[0]) + abs(gridButtons[2].loc[0])
    
    return xLenght, yLenght


def goHome(state=gripperOpen):
    velocity = 3
    acceleration = 3
    blend_1 = 0.0

    gripperControl(state)
    homeJoints = [1.6631979942321777, -1.1095922750285645, -2.049259662628174, 3.189222975368164, -0.6959036032306116, -9.445799001047405]

    # This i for moveL to home position
    #pose1 = [0.34, 0.34, 0.285, np.deg2rad(-84), np.deg2rad(35), np.deg2rad(-35)]
    #pose2 = [0, 0, 0, 0, 0, 0]

    #pose3 = rtde_c.poseTrans(pose1, pose2)

    #pose3.extend([velocity, acceleration, blend_1])

    #path = [pose3]

    rtde_c.moveJ(homeJoints, velocity, acceleration, blend_1)


def clickButton(pose1, velocity, acceleration, blend, bString):
    gripperControl(gripperClosed)
    buttonDepth = 0.01
    print("StringForClicks   ", bString)
    for j in range(len(bString)):
        currentTarget = bString[j]
        currentTarget = int(currentTarget)
        for i in buttonList:
            #If the first target matches the ArUco ID in the array then go to the location of the ArUco ID with 5 cm distance on the Z-axis
            if currentTarget == i.id:
                print(i.id)
                #Move to
                buttonLoc = [i.loc[0], i.loc[1], i.loc[2] + buttonDepth, 0.0, 0.0, 0.0]
                buttonLocBack = [i.loc[0], i.loc[1], i.loc[2] - buttonDepth, 0.0, 0.0, 0.0]

                buttonLocPush = rtde_c.poseTrans(pose1, buttonLoc)

                buttonLocPush.extend([velocity, acceleration, blend])

                buttonLocBack = rtde_c.poseTrans(pose1, buttonLocBack)

                buttonLocBack.extend([velocity, acceleration, blend])

                path = [buttonLocBack]
                
                rtde_c.moveL(path)  
                
                time.sleep(10)


def deg2rad(list):
    newlist = []
    for j in range(3):
        newlist.append(list[j])
    for i in range(3, 6):
        newlist.append(list[i]*pi/180)

    return newlist


def gridRun(pose1, velocity, acceleration, blend, xLenth, yLength): 
    
    boardNumber = 0

    for i in range(3):
        for j in range(3):
            x = j * xLenth
            y = i * yLength
            
            boardNumber += 1
            
            pose2 = [-xLenth + x, -yLength + y, 0, 0, 0, 0]
        
            pose3 = rtde_c.poseTrans(pose1, pose2)

            pose3.extend([velocity, acceleration, blend])

            path = [pose3]

            rtde_c.moveL(path)

            x_dist, y_dist, z_dist, ids = findArucoLocation()
           
            # Check for found buttons
            # Maybe add while loop here to ensure button detection before further movement
            if ids is not None and not isinstance(ids, str) and ids.any(): 
                buttonPos = buttonLocation(pose2, x_dist, y_dist, z_dist)
                button = buttonObject(ids[0][0], buttonPos, boardNumber)

                buttonList.append(button)

    for k in range(len(buttonList)):
        print(buttonList[k].id)


def ImuBoxTask(): 
    boardPoseRef, boardPose = scanImuBoardLoc(rtde_c, rtde_r)
    goToImuTable(rtde_c)
    findImuBox(rtde_c, rtde_r, gripperImuBox)
    goHome(state="imu")
    placeImu(boardPoseRef, boardPose, rtde_c, rtde_r, gripperOpen, imuAngle)
    

def boardTask(pose1):
    velocity = 0.33
    acceleration = 0.33
    blend_1 = 0.0

    goHome()

    xLength, yLength = getGridLength(pose1, velocity, acceleration, blend_1)

    gridRun(pose1, velocity, acceleration, blend_1, xLength, yLength)

    goHome()

    clickButton(pose1, velocity, acceleration, blend_1, buttonString)


def secretBoxTask(pose1):
    tableFitLoc = scanTable(rtde_c, rtde_r)
    #tableRefPose = [0.24614925086572445, 0.07873735284995985, 0.13874065786540948, np.deg2rad(-165.44384144), np.deg2rad(68.5), np.deg2rad(-0.59804425)]
    boxLoc = lidLocation(rtde_c, rtde_r)
    leaveLid, returnJoints, boxPosRefReturn = pickUpLid(rtde_c, rtde_r, boxLoc, gripperSecretLid, tableFitLoc, gripperOpen)
    secretId = scanSecretAruco(rtde_c, rtde_r)
    returnLid(rtde_c, rtde_r, leaveLid, gripperSecretLid, returnJoints, boxPosRefReturn, gripperOpen)
    goHome()
    velocity = 0.33
    acceleration = 0.33
    blend = 0
    rtde_c.setTcp([0, 0, 0.22, 0, 0, 0])
    secretId = str(secretId)

    clickButton(pose1, velocity, acceleration, blend, secretId)

def main():
    rtde_c.setTcp([0, 0, 0.22, 0, 0, 0])
    # Add wanted payload
    #rtde_c.setPayload(3.0, [0,0,0.22])
    pose1 = [0.34, 0.34, 0.285, np.deg2rad(-84), np.deg2rad(35), np.deg2rad(-35)]
    boardTask(pose1)

    goHome()

    #ImuBoxTask()

    #goHome()

    #secretBoxTask(pose1)

    #goHome()


if __name__ == "__main__":
    main()
