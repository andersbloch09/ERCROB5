import rtde_control
import rtde_receive
from math import pi
import numpy as np 
import multiprocessing
from main.gripperControl import gripperControl


IP = "192.168.1.102"

rtde_c = rtde_control.RTDEControlInterface(IP)
rtde_r = rtde_receive.RTDEReceiveInterface(IP)

gripperOpen = "open"
gripperClose = "close"

buttonString = "561"
buttonList = []

def interrupter():
    
class buttonObject():
    def __init__(self, id, loc, boardNumber):
        self.id = id 
        self.loc = loc
        self.boardNumber = boardNumber

def goHome():
    velocity = 3
    acceleration = 3
    blend_1 = 0.0
    gripperControl(gripperOpen)
    homeJoints = [1.6631979942321777, -1.1095922750285645, -2.049259662628174, 3.189222975368164, -0.6959036032306116, -9.445799001047405]

    # This i for moveL to home position
    #pose1 = [0.34, 0.34, 0.285, np.deg2rad(-84), np.deg2rad(35), np.deg2rad(-35)]
    #pose2 = [0, 0, 0, 0, 0, 0]

    #pose3 = rtde_c.poseTrans(pose1, pose2)

    #pose3.extend([velocity, acceleration, blend_1])

    #path = [pose3]

    rtde_c.moveJ(homeJoints, velocity, acceleration, blend_1)

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

    velocity = 0.33
    acceleration = 0.33
    blend_1 = 0.0

    offset = rtde_c.getTCPOffset()

    print(offset)

    rtde_c.setTcp([0, 0, 0.22, 0, 0, 0])
    # Add wanted payload
    #rtde_c.setPayload(3.0, [0,0,0.22])

    goHome()

    pose1 = [0.34, 0.34, 0.285, np.deg2rad(-84), np.deg2rad(35), np.deg2rad(-35)]



if __name__ == "__main__":
    # Create multiprocessing Events for signaling ArUco detection and completion
    aruco_detected_event = multiprocessing.Event()
    aruco_completed_event = multiprocessing.Event()

    # Create processes
    run_robot_process = multiprocessing.Process(target=main, args=(aruco_detected_event, aruco_completed_event))
    aruco_process = multiprocessing.Process(target=aruco_estimation, args=(aruco_detected_event, aruco_completed_event))

    # Start processes
    run_robot_process.start()
    aruco_process.start()

    # Wait for processes to finish
    run_robot_process.join()
    aruco_process.join()