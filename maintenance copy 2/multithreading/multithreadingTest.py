import rtde_control
import rtde_receive
from math import pi
import numpy as np 
from scipy.spatial.transform import Rotation
import time 
import cv2
import numpy as np
import multiprocessing
from multiprocessing import Pool






def run_robot(aruco_detected_event, aruco_completed_event):

    # Constants
    IP = "192.168.1.102"
    rtde_c = rtde_control.RTDEControlInterface(IP)
    rtde_r = rtde_receive.RTDEReceiveInterface(IP)
    velocity = 0.25
    acceleration = 0.25
    blend_1 = 0.0



    #The aruco IDs is given by the aruco detector
    aruco_id = [[7, 9, 3, 1, 8, 4, 5, 6, 2 ]]


    coordinates_for_button = [
    [-0.11, -0.185, 0, 0, 0, 0],
    [0.0, -0.185, 0, 0, 0, 0  ],
    [0.11, -0.185, 0, 0, 0, 0 ],
    [-0.11, 0.0, 0, 0, 0, 0   ],
    [0.0, 0.0, 0, 0, 0, 0     ],
    [0.11, 0.0, 0, 0, 0, 0    ],
    [-0.11, 0.185,   0, 0, 0, 0],
    [0.0, 0.185, 0,  0, 0, 0],
    [0.11, 0.185, 0, 0, 0, 0]
                            ]


    #Go to start position here
    path = []
    pose1 = [0.19, 0.19, 0.25, np.deg2rad(-84), np.deg2rad(35), np.deg2rad(-35)]
    poseHome = [-0.11, -0.185, 0, 0, 0, 0]
    pose3 = rtde_c.poseTrans(pose1, poseHome)
    pose3.extend([velocity, acceleration, blend_1])
    path.append(pose3)
    rtde_c.moveL([pose3])
    #??????



    # Create a dictionary to associate Aruco IDs with coordinates
    # The IDs can be acces with a for loop with the strings
    aruco_dict = dict(zip(aruco_id[0], coordinates_for_button))

    ##Example of accessing the dict 
    comp_order = [7, 9, 3, 1, 8, 4, 5, 6, 2 ]

    for i in range(len(comp_order)):
        
        requested_id = comp_order[i]
        coordinates = aruco_dict[requested_id]
        if requested_id in aruco_dict:
            coordinates = aruco_dict[requested_id]
            #print(f"Aruco ID: {requested_id}, Coordinates: {coordinates}")
        else:
            print(f"Aruco ID {requested_id} not found.")

        poseTrans = rtde_c.poseTrans(pose1, coordinates)
        poseTrans.extend([velocity, acceleration, blend_1])
        path.append(poseTrans)

    
    #Move the robot
    
    for i in range(len(comp_order)):
        print("Waiting for ArUco detection...")
        aruco_detected_event.wait()  # Wait until aruco_estimation signals detection
        aruco_detected_event.clear()  # Reset the event for the next iteration

        print(f"Starting robot movement for ArUco ID {comp_order[i]}...")
        
        rtde_c.moveL([path[i]])
        # Signal completion to aruco_estimation
        aruco_completed_event.set()

    #rtde_c.moveL(path)
    rtde_c.stopScript()



def aruco_estimation(aruco_detected_event, aruco_completed_event):
    # Load camera calibration parameters
    #Camera Matrix
    mtx = np.array([[1.42954906e+03, 0.00000000e+00, 6.82253318e+02],
                [0.00000000e+00, 1.42214137e+03, 5.23272094e+02],
                [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
    #Distortion matrix 
    dist = np.array([[6.37038859e-03, 9.72517633e-01, 4.36392732e-03, -6.79299631e-04, -3.2314831]])
    # Known size of the ArUco marker in real-world units (e.g., in meters)
    aruco_marker_size = 0.025 # Adjust this based on the actual size of your ArUco marker
    # ArUco dictionary and parameters
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    parameters = cv2.aruco.DetectorParameters()

    # Open a connection to the camera (adjust the index as needed, typically 0 or 1)
    cap = cv2.VideoCapture(2)
    id_list = []


    #Loop----------------------
    while True:
        # Read a frame from the camera
        ret, frame = cap.read()

        # Convert the frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None:
            # Draw the detected markers on the frame
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            # Estimate the distance to each detected marker
            for i in range(len(ids)):
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], aruco_marker_size, mtx, dist)

                # Extract the translation vector (distance) along the z-axis
                distance = tvec[0, 0, 2]

                # Display the distance for each marker
                #cv2.putText(frame, f"Marker {ids[i][0]} Distance: {distance:.2f} meters", (10, 30 + i * 30),
                #            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                #print("Distance",round(100*distance), "cm")
                #print("ID",ids[i][0])

                if ids[i][0] not in id_list:
                    id_list.append(ids[i][0])
                    if aruco_detected_event.set():  # Signal to run_robot that ArUco is detected
                        print(f"Waiting for robot movement to complete for ArUco ID {ids[i][0]}...")
                        aruco_completed_event.wait()  # Wait until run_robot signals completion
                        aruco_completed_event.clear()  # Reset the event for the next iteration


                #Test matrix
                # 6, 4 12
                # 14 8 1 
                # 11 9 7



        # Display the frame
        cv2.imshow('Distance Estimation to ArUco Markers', frame)

        # Break the loop if 'ESC' is pressed
        if cv2.waitKey(1) & 0xFF == 27:
            break

        #glob_distance = distance
        #glob_id = ids[i][0]

    # Release the camera
    print(id_list)
    cap.release()
    cv2.destroyAllWindows()




# Create multiprocessing Events for signaling ArUco detection and completion
aruco_detected_event = multiprocessing.Event()
aruco_completed_event = multiprocessing.Event()

# Create processes
run_robot_process = multiprocessing.Process(target=run_robot, args=(aruco_detected_event, aruco_completed_event))
aruco_process = multiprocessing.Process(target=aruco_estimation, args=(aruco_detected_event, aruco_completed_event))

# Start processes
run_robot_process.start()
aruco_process.start()

# Wait for processes to finish
run_robot_process.join()
aruco_process.join()