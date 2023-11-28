import rtde_control
import rtde_receive
from math import pi
import numpy as np 
from mainFolder.CameraOffset import buttonLocation
from mainFolder.ArucoEstimation import findArucoLocation
from multithreading.ArucoEstimationFor_moveStopTest import findArucoLocation_moveStopTest
from mainFolder.gripperControl import gripperControl

IP = "192.168.1.102"

rtde_c = rtde_control.RTDEControlInterface(IP)
rtde_r = rtde_receive.RTDEReceiveInterface(IP)

gripperOpen = "open"
gripperClosed = "close"

buttonString = "561"
buttonList = []

class buttonObject():
    def __init__(self, id, loc, boardNumber):
        self.id = id 
        self.loc = loc
        self.boardNumber = boardNumber

def getGridLength(homeBoardPose):
    velocity = 0.1
    acceleration = 0.1
    blend = 0.0

    gridButtons = []
    zeroPose = [0, 0, 0, 0, 0, 0]
    
    y = 0.5
    x = 0.3

    # Add center button to gridButtons for reference
    x_dist, y_dist, z_dist, ids = findArucoLocation()
    buttonPos = buttonLocation(zeroPose, x_dist, y_dist, z_dist)
    centerRef = buttonObject(ids[0][0], buttonPos, 4)

    gridButtons.append(centerRef)
    
    pose2 = [0, y, 0, 0, 0, 0]

    poseFound = rtde_c.poseTrans(homeBoardPose, pose2)
        
    poseFound.extend([acceleration, velocity, blend])

    path = [poseFound]
    
    rtde_c.moveL(path, asynchronous = True)

    while len(gridButtons) < 2:
        x_dist, y_dist, z_dist, ids = findArucoLocation_moveStopTest()
        
        # Check for found buttons
        if ids is not None and not isinstance(ids, str) and ids.any():
                if gridButtons[0].id != ids[0][0]:
                    pose2 = rtde_r.getActualTCPPose()
                    rtde_c.stopL(a=2.0, asynchronous = False)
                    buttonPos = buttonLocation(pose2, x_dist, y_dist, z_dist)
                    topRef = buttonObject(ids[0][0], buttonPos, 4)

                    gridButtons.append(topRef)
                    break
    
    pose2[0] = x

    pose2 = rtde_c.poseTrans(homeBoardPose, pose2)

    pose2.extend([acceleration, velocity, blend])

    path = [pose2]

    rtde_c.moveL(path, asynchronous = True)
    
    while len(gridButtons) < 3:
        x_dist, y_dist, z_dist, ids = findArucoLocation()
        
        # Check for found buttons
        if ids is not None and not isinstance(ids, str) and ids.any():
            if gridButtons[1].id != ids[0][0]:
                pose2 = rtde_r.getActualTCPPose()
                rtde_c.stopL(a=2.0, asynchronous = False)
                buttonPos = buttonLocation(pose2, x_dist, y_dist, z_dist)
                leftRef = buttonObject(ids[0][0], buttonPos, 4)

                gridButtons.append(leftRef)
                break
        

    yLenght = abs(gridButtons[0].loc[1]) + abs(gridButtons[1].loc[1])
    xLenght = abs(gridButtons[1].loc[0]) + abs(gridButtons[2].loc[0])
    
    return xLenght, yLenght


def goHome():
    velocity = 3
    acceleration = 3
    blend_1 = 0.0

    gripperControl(gripperOpen)
    homeJoints = [1.6631979942321777, -1.1095922750285645, -2.049259662628174, 3.189222975368164, -0.6959036032306116, -9.445799001047405]

    # This i for moveL to home position
    #homeBoardPose = [0.34, 0.34, 0.285, np.deg2rad(-84), np.deg2rad(35), np.deg2rad(-35)]
    #pose2 = [0, 0, 0, 0, 0, 0]

    #pose3 = rtde_c.poseTrans(homeBoardPose, pose2)

    #pose3.extend([acceleration, velocity, blend_1])

    #path = [pose3]

    rtde_c.moveJ(homeJoints, velocity, acceleration)

def clickButton(homeBoardPose, velocity, acceleration, blend):
    gripperControl(gripperClosed)
    buttonDepth = 0.009

    for j in range(len(buttonString)):
        currentTarget = buttonString[j]
        currentTarget = int(currentTarget)
        for i in buttonList:
            #If the first target matches the ArUco ID in the array then go to the location of the ArUco ID with 5 cm distance on the Z-axis
            if currentTarget == i.id:
                print(i.id)
                #Move to
                buttonLoc = [i.loc[0], i.loc[1], i.loc[2] + buttonDepth, 0.0, 0.0, 0.0]
                buttonLocBack = [i.loc[0], i.loc[1], i.loc[2] - buttonDepth, 0.0, 0.0, 0.0]

                buttonLocPush = rtde_c.poseTrans(homeBoardPose, buttonLoc)

                buttonLocPush.extend([velocity, acceleration, blend])

                buttonLocBack = rtde_c.poseTrans(homeBoardPose, buttonLocBack)

                buttonLocBack.extend([velocity, acceleration, blend])

                path = [buttonLocBack, buttonLocPush, buttonLocBack]
                
                rtde_c.moveL(path)


def rad2deg(list):
    newlist = []
    for j in range(3):
        newlist.append(list[j])
    for i in range(3, 6):
        newlist.append(list[i] * (180/pi))

    return newlist

def deg2rad(list):
    newlist = []
    for j in range(3):
        newlist.append(list[j])
    for i in range(3, 6):
        newlist.append(list[i]*pi/180)

    return newlist

def gridRun(homeBoardPose, velocity, acceleration, blend, xLenth, yLength): 
    
    boardNumber = 0

    for i in range(3):
        for j in range(3):
            x = j * xLenth
            y = i * yLength
            
            boardNumber += 1
            
            pose2 = [-xLenth + x, -yLength + y, 0, 0, 0, 0]
        
            pose3 = rtde_c.poseTrans(homeBoardPose, pose2)

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
            


def main():
    tcp_pose = rtde_r.getActualTCPPose()

    tcp_pose = rad2deg(tcp_pose)

    print(tcp_pose)

    velocity = 0.33
    acceleration = 0.33
    blend_1 = 0.0

    offset = rtde_c.getTCPOffset()

    print(offset)

    rtde_c.setTcp([0, 0, 0.22, 0, 0, 0])
    # Add wanted payload
    #rtde_c.setPayload(3.0, [0,0,0.22])

    goHome()

    homeBoardPose = [0.34, 0.34, 0.285, np.deg2rad(-84), np.deg2rad(35), np.deg2rad(-35)]

    xLenth, yLength = getGridLength(homeBoardPose, velocity, acceleration, blend_1)

    gridRun(homeBoardPose, velocity, acceleration, blend_1, xLenth, yLength)

    goHome()

    clickButton(homeBoardPose, velocity, acceleration, blend_1)

    goHome()

if __name__ == "__main__":
    main()
