import rtde_control
import rtde_receive
from math import pi
import numpy as np

IP = "192.168.1.102"

rtde_c = rtde_control.RTDEControlInterface(IP)
rtde_r = rtde_receive.RTDEReceiveInterface(IP)


def goHome():
    velocity = 3
    acceleration = 3
    blend_1 = 0.0

    homeJoints = [1.6631979942321777, -1.1095922750285645, -2.049259662628174, 3.189222975368164, -0.6959036032306116, -9.445799001047405]

    # This i for moveL to home position
    #pose1 = [0.34, 0.34, 0.285, np.deg2rad(-84), np.deg2rad(35), np.deg2rad(-35)]
    #pose2 = [0, 0, 0, 0, 0, 0]

    #pose3 = rtde_c.poseTrans(pose1, pose2)

    #pose3.extend([velocity, acceleration, blend_1])

    #path = [pose3]

    rtde_c.moveJ(homeJoints, velocity, acceleration, blend_1)

velocity = 0.33
acceleration = 0.33
blend = 0.0

offset = rtde_c.getTCPOffset()

print(offset)

rtde_c.setTcp([0, 0, 0.22, 0, 0, 0])
# Add wanted payload
#rtde_c.setPayload(3.0, [0,0,0.22])

goHome()

pose1 = [0.34, 0.34, 0.285, np.deg2rad(-86.7), np.deg2rad(23.2), np.deg2rad(-23.6)]

pose2 = [0, 0, 0, 0, 0, 0]
poseFound = rtde_c.poseTrans(pose1, pose2)

poseFound.extend([velocity, acceleration, blend])

path = [poseFound]

rtde_c.moveL(path)