import numpy as np 
from . import gripperControl
from . import ArucoEstimationSmall as ArucoEstS
from . import ArucoEstimation as ArucoEst
from scipy.spatial.transform import Rotation
from . import CameraOffset as co
from math import pi
import time

def scanTable():
    tableHomeJ = [0.8427166938781738, -2.0175072155394496, -0.7320303916931152, 1.1957790094562988, -1.5637462774859827, -9.366041310617717]

    