import numpy as np

def buttonLocation(currentRobotLocation, x_initial, y_initial, z_initial):
    # Initial point in the camera frame
    # Translation to a new frame
    translationVectorNewFrame = np.array([0, -0.01, -0.17])

    arucoToButton = np.array([0, 0.06, -0.03])

    # Apply the translation to get the final position in the new frame
    #point_final_new_frame = point_final_camera_frame + translation_vector_new_frame
    currentRobotLocation = np.array([currentRobotLocation[0], currentRobotLocation[1], currentRobotLocation[2]])
    point_final_new_frame =  np.array([x_initial, y_initial, z_initial]) + translationVectorNewFrame + currentRobotLocation

    buttonPos = point_final_new_frame + arucoToButton

    print("Final Point in New Frame after applying displacement:", point_final_new_frame)
    print("Button location:", buttonPos[0],",",buttonPos[1],",",buttonPos[2])
    return buttonPos

def imuBoxLocationPickup(z_initial):
    translationVectorNewFrame = np.array([-0.17])
    grabTranslation = np.array([0.06])

    point_final_new_frame =  np.array([z_initial]) + translationVectorNewFrame + grabTranslation
    print("z_DIST", point_final_new_frame)
    return point_final_new_frame

def imuBoxLocationFit(x_initial, y_initial, z_initial):
    translationVectorNewFrame = np.array([0, 0.02, -0.17])

    boxFitLoc =  np.array([x_initial, y_initial, z_initial]) + translationVectorNewFrame

    print("Box Fit Loc", boxFitLoc)

    return boxFitLoc

def arucoToBoardImu(x_initial, y_initial, z_initial):
    translationVectorNewFrame = np.array([0, -0.01, -0.17])
    boardCenter = np.array([0.09, 0.15, 0])

    boardCenterImu =  np.array([x_initial, y_initial, z_initial]) + translationVectorNewFrame + boardCenter

    print("BoardCenterImu", boardCenterImu)

    return boardCenterImu

def lidTableLoc(x_initial, y_initial, z_initial):
    translationVectorNewFrame = np.array([0, -0.01, -0.17])
    tableCenter = np.array([0.07, 0.085, 0])

    lidPlacement =  np.array([x_initial, y_initial, z_initial]) + translationVectorNewFrame + tableCenter

    print("lidPlacement", lidPlacement)

    return lidPlacement

def lidBoxLoc(x_initial, y_initial, z_initial):
    translationVectorNewFrame = np.array([0, -0.01, -0.17])
    lidCenter = np.array([0.0, -0.075, 0.055])

    lidBoxLocation =  np.array([x_initial, y_initial, z_initial]) + translationVectorNewFrame + lidCenter

    print("lid location", lidBoxLocation)

    return lidBoxLocation
