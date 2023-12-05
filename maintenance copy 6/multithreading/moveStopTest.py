import rtde_control
import rtde_receive
from math import pi
import numpy as np 
import threading
from ArucoEstimationFor_moveStopTest import findArucoLocation_moveStopTest
import time


IP = "192.168.56.101"

rtde_c = rtde_control.RTDEControlInterface(IP)
rtde_r = rtde_receive.RTDEReceiveInterface(IP)

gripperOpen = "open"
gripperClose = "close"

buttonString = "561"
buttonList = []
    
class buttonObject():
    def __init__(self, id, loc, boardNumber):
        self.id = id 
        self.loc = loc
        self.boardNumber = boardNumber

def goHome():
    velocity = 3
    acceleration = 3
    blend_1 = 0.0
    #gripperControl(gripperOpen)
    homeJoints = [1.6631979942321777, -1.1095922750285645, -2.049259662628174, 3.189222975368164, -0.6959036032306116, -9.445799001047405]

    # This i for moveL to home position
    #pose1 = [0.34, 0.34, 0.285, np.deg2rad(-84), np.deg2rad(35), np.deg2rad(-35)]
    #pose2 = [0, 0, 0, 0, 0, 0]

    #pose3 = rtde_c.poseTrans(pose1, pose2)

    #pose3.extend([velocity, acceleration, blend_1])

    #path = [pose3]
    rtde_c.moveJ(homeJoints, velocity, acceleration, asynchronous = False)
    

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


def main():
    tcp_pose = rtde_r.getActualTCPPose()

    tcp_pose = rad2deg(tcp_pose)

    print(tcp_pose)

    velocity = 0.2
    acceleration = 0.2
    blend_1 = 0.0

    rtde_c.setTcp([0, 0, 0.0, 0, 0, 0])
    newPosition = rtde_r.getActualQ()
    print(newPosition)
    # Add wanted payload
    #rtde_c.setPayload(3.0, [0,0,0.22])

    pose1 = [0.34, 0.34, 0.285, np.deg2rad(-84), np.deg2rad(35), np.deg2rad(-35)]
    pose2 = [0, 0, 0, 0, 0, 0]

    pose3 = rtde_c.poseTrans(pose1, pose2)

    pose3.extend([velocity, acceleration, blend_1])

    newPosition = [pose3]


    
    #newPosition = [2.331293124080819, -1.810071159921117, -1.5096448437880934, 3.9344636379772604, -0.033758323199299056, -10.036966192907858]
    while True: 
        goHome()
        rtde_c.moveL(newPosition, asynchronous = True)
        while True:
            x_dist, y_dist, z_dist, ids = findArucoLocation_moveStopTest()
            
            if ids is not None and not isinstance(ids, str) and ids.any(): 
                rtde_c.stopJ(a=2.0, asynchronous = False)
                print(x_dist, y_dist, z_dist, ids)
                break



if __name__ == "__main__":
    main()