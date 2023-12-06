import rtde_control
import rtde_receive
from math import pi
import numpy as np
from mainFolder.CameraOffset import buttonLocation
from mainFolder.ArucoEstimation import findArucoLocation
from mainFolder.gripperControl import gripperControl
from mainFolder.imuBoxMovement import (goToImuTable, findImuBox,
                                       scanImuBoardLoc, placeImu)
from mainFolder.secretBoxMovement import (scanTable, lidLocation, pickUpLid,
                                          scanSecretAruco, returnLid)

# Ip of the robot
IP = "192.168.1.102"

# This connects the controllers and creates the usable variable
rtde_c = rtde_control.RTDEControlInterface(IP)
rtde_r = rtde_receive.RTDEReceiveInterface(IP)

# Different gripper states
gripperOpen = "open"
gripperClosed = "close"
gripperImuBox = "imu"
gripperSecretLid = "secretLid"

# List for button objects on the button board
buttonList = []

# Input string for competition
buttonString = "3976"

# Input angle
imuAngle = 53


# Button class for the button board
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
    homeJoints = [1.6631979942321777, -1.1095922750285645, -2.049259662628174,
                  3.189222975368164, -0.6959036032306116, -9.445799001047405]

    rtde_c.moveJ(homeJoints, velocity, acceleration, blend_1)


def clickButton(pose1, velocity, acceleration, blend, bString):
    gripperControl(gripperClosed)
    buttonDepth = 0.01
    print("StringForClicks   ", bString)
    for j in range(len(bString)):
        currentTarget = bString[j]
        currentTarget = int(currentTarget)
        for i in buttonList:
            # If the first target matches the ArUco ID in the array
            # then go to the location of the ArUco ID
            # with 5 cm distance on the Z-axis
            if currentTarget == i.id:
                print(i.id)
                # Move to
                buttonLoc = [i.loc[0], i.loc[1],
                             i.loc[2] + buttonDepth, 0.0, 0.0, 0.0]
                buttonLocBack = [i.loc[0], i.loc[1],
                                 i.loc[2] - buttonDepth, 0.0, 0.0, 0.0]

                buttonLocPush = rtde_c.poseTrans(pose1, buttonLoc)

                buttonLocPush.extend([velocity, acceleration, blend])

                buttonLocBack = rtde_c.poseTrans(pose1, buttonLocBack)

                buttonLocBack.extend([velocity, acceleration, blend])

                path = [buttonLocBack, buttonLocPush, buttonLocBack]

                rtde_c.moveL(path)


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
            if ids is not None and not isinstance(ids, str) and ids.any():
                buttonPos = buttonLocation(pose2, x_dist, y_dist, z_dist)
                button = buttonObject(ids[0][0], buttonPos, boardNumber)

                buttonList.append(button)

    for k in range(len(buttonList)):
        print(buttonList[k].id)


def ImuBoxTask():
    # scanImuBoardLoc scans the imu board to find the location to put the imu
    # before picking it up
    boardPoseRef, boardPose = scanImuBoardLoc(rtde_c, rtde_r)
    # Joint states to rotate to a position where the imu can be scanned
    goToImuTable(rtde_c)
    # finds the imu box and rotates the robot to fit and pick up the imu
    findImuBox(rtde_c, rtde_r, gripperImuBox)
    # keeps the state as imu to not drop it
    goHome(state="imu")
    # The imu is placed and the robot will return to home after
    placeImu(boardPoseRef, boardPose, rtde_c, rtde_r, gripperOpen, imuAngle)


def boardTask(pose1):
    velocity = 0.33
    acceleration = 0.33
    blend_1 = 0.0

    goHome()
    # This function creates the grid lengths
    # to make a grid run afterwards finding the arucos
    xLength, yLength = getGridLength(pose1, velocity, acceleration, blend_1)

    # gridRun runs through a 3x3 grid bassed on the lengths found
    # in the function above
    gridRun(pose1, velocity, acceleration, blend_1, xLength, yLength)

    goHome()

    # This function clicks the buttons given in the competition string
    clickButton(pose1, velocity, acceleration, blend_1, buttonString)


def secretBoxTask(pose1):
    # This function scans the table first to know where to put the lid
    tableFitLoc = scanTable(rtde_c, rtde_r)

    # This function scans the secret box and returns the location of the lid
    # based on a translation
    boxLoc = lidLocation(rtde_c, rtde_r)

    # Here the location is used to pick up the lid and place it on the aruco
    leaveLid, returnJoints, boxPosRefReturn = pickUpLid(
        rtde_c, rtde_r, boxLoc, gripperSecretLid, tableFitLoc, gripperOpen)

    # The secret ID is scanned and returned in this function
    secretId = scanSecretAruco(rtde_c, rtde_r)

    # The lid is put back on
    returnLid(rtde_c, rtde_r, leaveLid, gripperSecretLid,
              returnJoints, boxPosRefReturn, gripperOpen)

    goHome()

    velocity = 0.33
    acceleration = 0.33
    blend = 0
    rtde_c.setTcp([0, 0, 0.22, 0, 0, 0])
    secretId = str(secretId)
    # The button with the secret ID is pressed on the board
    clickButton(pose1, velocity, acceleration, blend, secretId)


def main():
    # Offset made for the gripper fully closed
    rtde_c.setTcp([0, 0, 0.22, 0, 0, 0])

    # This is the cartesian point for the home position
    # Used to move relative to the frame made there
    pose1 = [0.34, 0.34, 0.285,
             np.deg2rad(-84), np.deg2rad(35), np.deg2rad(-35)]

    boardTask(pose1)

    goHome()

    ImuBoxTask()

    goHome()

    secretBoxTask(pose1)

    goHome()


if __name__ == "__main__":
    main()
