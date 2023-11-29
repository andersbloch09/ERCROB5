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

def imuBoxLocation(x_initial, y_initial, z_initial, z_rot):
    translationVectorNewFrame = np.array([0, -0.01, -0.17])
    grabLocation = np.array([0, 0, 0.06])

    point_final_new_frame =  np.array([x_initial, y_initial, z_initial]) + translationVectorNewFrame + grabLocation

    print(point_final_new_frame)