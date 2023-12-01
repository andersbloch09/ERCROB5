import numpy as np 
from . import gripperControl
from . import ArucoEstimationSmall as ArucoEstS
from . import ArucoEstimation as ArucoEst
from . import CameraOffset as co
from math import pi
import time

def scanTable(rtde_c, rtde_r):
    velocity = 1
    acceleration = 0.1
    blend_1 = 0.0

    tableHomeJ = [0.8427166938781738, -2.0175072155394496, -0.7320303916931152, 1.1957790094562988, -1.5637462774859827, -9.366041310617717]
    rtde_c.moveJ(tableHomeJ, velocity, acceleration, blend_1)
    
    tableFitLoc = []

    while len(tableFitLoc) < 1:
        x_distance, y_distance, z_distance, ids = ArucoEst.findArucoLocation()
        if ids is not None and not isinstance(ids, str) and ids.any():
            tableFitLoc = co.lidTalbeLoc(x_distance, y_distance, z_distance)
    
    return tableFitLoc