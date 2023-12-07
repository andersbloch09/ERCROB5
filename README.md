# ERCROB5
Contains code used on a UR3e and ExoMy for European Rover Challenge. 

## Maintenance
The code for the maintenance task of the project can be found in the default branch.
The folder seperate the code into the different tests and the main code can be found in final test main.py. 

## Navigation
The code for the navigation task can be found in the navigation branch.

The ExoMy_Software folder contains all the pre-made ROS files from the ESA. Inside that folder is the "Melodic" folder, which contains a custom docker and ros workspace which has been developed by us.
Navigate to the Aruco_Package/src folder, where Planner.py and Aruco.py can be found. Aruco.py is the node that perfoms localization of aruco codes, and planner.py is the pathplanner and executer
