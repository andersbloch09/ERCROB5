import cv2
import numpy as np
import time


def findArucoLocation():
    # Load camera calibration parameters
    mtx = np.array([[1.44003309e+03, 0.00000000e+00, 6.86010223e+02],
                    [0.00000000e+00, 1.43870157e+03, 4.31888568e+02],
                    [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

    dist = np.array([[7.29890166e-02, -7.14335748e-01, 1.44494297e-02,
                      9.08325864e-04, 7.15318943e+00]])

    # Known size of the ArUco marker in real-world units (e.g., in meters)
    # Adjust this based on the actual size of your ArUco marker
    aruco_marker_size = 0.04

    # ArUco dictionary and parameters
    aruco_dict = cv2.aruco.getPredefinedDictionary(
        cv2.aruco.DICT_ARUCO_ORIGINAL)
    parameters = cv2.aruco.DetectorParameters()

    # Open a connection to the camera
    # (adjust the index as needed, typically 0 or 1)
    cap = cv2.VideoCapture(2)

    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)

    # Set the video capture object to use the maximum resolution
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    checkTime = time.time()
    startTime = time.time()
    while True:

        checkTime = time.time()

        # Read a frame from the camera
        ret, frame = cap.read()

        # Convert the frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers
        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray, aruco_dict, parameters=parameters)

        if ids is not None:
            # Draw the detected markers on the frame
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            # Estimate the distance to each detected marker
            for i in range(len(ids)):
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners[i], aruco_marker_size, mtx, dist)

                # Extract the translation vector (distance) along the z-axis
                z_distance = tvec[0, 0, 2]
                y_distance = tvec[0, 0, 1]
                x_distance = tvec[0, 0, 0]

                # This finds the rotation matrix used to calculate
                # the euler values for the rotation
                values = cv2.Rodrigues(rvec)
                rotation_matrix = values[0]

                # Display the distance for each marker
                cv2.putText(frame, f"Marker {ids[i][0]} Distance:
                            {z_distance: .2f} meters", (10, 30 + i * 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                print("Distance", round(100*z_distance), "cm")
                print("x coordinate", x_distance)
                print("y coordinate", y_distance)

                print("ID", ids[i][0])

                cv2.drawFrameAxes(frame, mtx, dist, rvec[0], tvec[0], 0.1)

                if tvec.any():
                    return x_distance, y_distance, \
                            z_distance, ids, rotation_matrix
        else:
            x_distance, y_distance, z_distance, ids = 0, 0, 0, ""
            rotation_matrix = []
            return x_distance, y_distance, z_distance, ids, rotation_matrix
