#!/usr/bin/env python

import rospy
from AruCo_Package.msg import ArucoData
from nav_msgs.msg import Odometry
from AruCo_Package.msg import RoverCommand
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
import numpy as np
from tf.transformations import euler_from_quaternion
from math import degrees
from math import atan2
from math import sqrt
from heapq import heappop, heappush
from time import sleep

class Handler:
    def __init__(self):
        rospy.init_node('handler_node', anonymous=True)
        self.marker_data = {-1 : (100,100)}  # Dictionary to hold ID-Translation pairs
        self.other_markers = {-1 : (100,100)} # Dictionary to hold all ID-translation pairs, except the current goal
        self.visit_order = [2,1,3,6,6]  # Example visit order
        self.current_goal = None
        self.path = None
        self.refined_path = None
        self.goal_reached = True
        self.next_goal = None
        self.HOME = False
        self.homePathGenerated = False
        self.lookAhead = 0.1 # Look Ahead distance for Pure Pursuit
        self.avoidanceDist = 0.7 # Avoidance distance for pathplanning
        
        # Initialize the subscribers
        self.aruco_sub = rospy.Subscriber('/aruco_data', ArucoData, self.aruco_callback)
        self.odom_sub = rospy.Subscriber('/t265/odom/sample', Odometry, self.odom_callback)
        self.driver_sub = rospy.Subscriber('/t265/accel/sample', Imu, self.driver_callback)

        # Initialize publisher
        self.rover_command_pub = rospy.Publisher("/rover_command", RoverCommand, queue_size=10)

    def driver_callback(self,temp):
        # Independent callback function that uses the refresh rate of the /t265/accel/sample topic to execute relevant code without hindering
        # the update of odometry- and arucodata. (Kind of a hack)
        self.Initiate()

    def aruco_callback(self, msg):
        # Apply coordinate transformation to incoming ArucoData- This is now in the local frame
        # This conversion ensures the coincidence of the representing frames for the Aruco-detector and odometry data.
        transformed_translation = self.transform_coordinates(msg.translation)

        # Convert to global frame:
        transformed_translation = transform_to_global_frame(transformed_translation.x,transformed_translation.y,self.x, self.y, self.euler_angles[2])

        # Determine confidence interval:
        dist_to_marker = distance((self.x, self.y), (transformed_translation[0],transformed_translation[1]))
        print(f"Distance to Marker no. {msg.id} is {dist_to_marker}")
        # Append confidence level according to distance
        if dist_to_marker < 2.5:
            confidence = 1    
        else:
            confidence = 2.5/dist_to_marker
            
        
        # Store transformed coordinates with the corresponding ID and confidence interval
        transformed_translation[0] = round(transformed_translation[0],2)
        transformed_translation[1] = round(transformed_translation[1],2)
        transformed_translation.append(round(confidence,2))

        # Ensure that only relevant Aruco ID's are stored:
        if 11 > msg.id > 0:
            if msg.id not in self.marker_data:
                self.marker_data[msg.id] = transformed_translation
            # Replace old positional data for marker, if confidence is higher
            elif confidence > self.marker_data[msg.id][2]:
                self.marker_data[msg.id] = transformed_translation
                print("Higher Confidence Level Found!")

        #print(self.marker_data)
        
    def odom_callback(self, data):
        # Callback function that stores odometry data from T265 as appropriate local values
        self.x = round(data.pose.pose.position.x,2)
        self.y = round(data.pose.pose.position.y,2)
        self.z = round(data.pose.pose.position.z,2)

        # Extract orientation (quaternion) from the Odometry message
        orientation_quaternion = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w
        )

        # Convert quaternion to Euler angles in radians
        self.euler_angles = euler_from_quaternion(orientation_quaternion)

        # Convert Euler angles from radians to degrees
        self.roll = degrees(self.euler_angles[0])
        self.pitch = degrees(self.euler_angles[1])
        self.yaw = degrees(self.euler_angles[2])
       
    def Initiate(self):
        # Function that performs all path planning and execution
        print(f"Stats: {self.visit_order} and {self.current_goal}")
        # Ensure that the visit_order list still contains goals and that the rover has no current goal
        if self.visit_order and not self.current_goal:
            print("Looking for target")
            # If a goal has just been reached, pop next goal in the visit order list
            if self.goal_reached:
                print("Popping Next Goal")
                print(self.marker_data)
                self.next_goal = self.visit_order.pop(0)
                self.goal_reached = False
            # If rover has a new goal, it exists in the Marker_data dictionary and no path has been generated yet: Generate path
            if self.next_goal in self.marker_data and not self.refined_path:
                print("Generating Path")
                self.current_goal = self.marker_data[self.next_goal]
                self.other_markers = self.marker_data
                self.other_markers.pop(self.next_goal)
                self.refined_path = generate_path((self.x, self.y), self.current_goal, self.other_markers)
            # If rover has a new goal, but it doesn't exist in the marker_data dictionary, perform "Searching" maneuvre
            elif self.next_goal not in self.marker_data:
                self.searching()
            else:
                print("Initiation Error")
        # If there are no more goals left in Visit_order, and the final goal has been reached, perform "returnHome" maneuvre
        elif not self.visit_order and self.goal_reached:
            #If rover is not home yet
            if not self.HOME:   
                self.returnHome()
                
        # If the rover has a current goal, and the path has been generated, begin movement
        if self.current_goal and self.refined_path:
            print(f"Current Goal: {self.current_goal}")
            self.move_to_next_point()
    
    def returnHome(self):
        # Generate Path Home
        print("Returning Home")
        # Generate path home only once
        if not self.homePathGenerated:
            self.refined_path = generate_path((self.x, self.y), (0,0), self.marker_data)
            self.homePathGenerated = True
        self.move_to_next_point()
        # Determine whether the goal has been reached with a tolerance of 10cm
        if distance((self.x,self.y),(0,0)) < 0.1:
                self.publish_movement(0,0,2)
                self.HOME = True

    def move_to_next_point(self):
        #print(self.refined_path)
        # First goal-point in our "pure pursuit" is set, and the distance to it is checked
        current_point = self.refined_path[0]
        distance_to_current = distance((self.x,self.y), current_point)
        if distance_to_current < self.lookAhead:  # Assuming a threshold for reaching the point
            # Remove point if it is too close
            self.refined_path.pop(0)
            # If path is "completed", set current goal and goal reached, so that self.initate() plans path to next point
            if not self.refined_path:
                self.current_goal = None
                self.goal_reached = True
            # Else set the new goal
            else:
                current_point = self.refined_path[0]
        
        # Go to current point
        goto(current_point[0],current_point[1])

    def rotate_x(self, angle):
        # Rotation matrix around the x-axis
        return np.array([
            [1, 0, 0],
            [0, np.cos(angle), -np.sin(angle)],
            [0, np.sin(angle), np.cos(angle)]
        ])

    def rotate_y(self, angle):
        # Rotation matrix around the y-axis
        return np.array([
            [np.cos(angle), 0, np.sin(angle)],
            [0, 1, 0],
            [-np.sin(angle), 0, np.cos(angle)]
        ])
    
    def rotate_z(self, angle):
        # Rotation matrix around the z-axis
        return np.array([
            [np.cos(angle), -np.sin(angle), 0],
            [np.sin(angle), np.cos(angle), 0],
            [0, 0, 1]
        ])

    def transform_coordinates(self, translation):
        # Apply rotations to transform coordinates
        rotation_y = self.rotate_y(np.pi / 2)   # +90 degrees around new y-axis
        rotation_z = self.rotate_z(-np.pi / 2)
        # Convert translation vector to a NumPy array
        translation_vector = np.array([0.01*translation.x, 0.01*translation.y, 0.01*translation.z])

        # Apply rotations sequentially
        transformed_coords = np.dot(rotation_z, translation_vector)
        transformed_coords = np.dot(rotation_y, transformed_coords)
             

        return Vector3(*transformed_coords)
        
    def publish_movement(self, velocity, angle, mode):
        # Publish movement commands for the rover:
        command = RoverCommand()
        command.steering = angle
        command.locomotion_mode = mode
        command.motors_enabled = True
        command.connected = True
        command.vel = velocity
        
        self.rover_command_pub.publish(command)

    def searching(self):
        # Searching maneuvre is incremental turning
        self.publish_movement(1,180,2)
        sleep(0.1)
        self.publish_movement(0,180,2)
        sleep(0.4)
         
def transform_to_local_frame(x_g, y_g, robot_x, robot_y, rotation_z):
    # Calculate the inverse transformation
    delta_x = x_g - robot_x
    delta_y = y_g - robot_y
    cos_theta = np.cos(rotation_z)
    sin_theta = np.sin(rotation_z)

    x_l = (delta_x * cos_theta + delta_y * sin_theta)
    y_l = (-delta_x * sin_theta + delta_y * cos_theta)

    # Return the transformed point in the local frame
    return [x_l, y_l]

def return_angle(x,y):
    return degrees(atan2(y,x))

def transform_to_global_frame(x_l, y_l, robot_x, robot_y, rotation_z):
    # Apply the transformation
    x_g = robot_x + (x_l * np.cos(rotation_z)) - (y_l * np.sin(rotation_z))
    y_g = robot_y + (x_l * np.sin(rotation_z)) + (y_l * np.cos(rotation_z))

    # Return the transformed point in the global frame
    return [x_g, y_g]

def goto(x,y):
    global coordinate_handler
    print(f"Entered GoTo: x: {x}, y:{y}")
    print(f"Current Pos: x: {coordinate_handler.x}, y:{coordinate_handler.y}")
    print(f"Next Goal: {coordinate_handler.next_goal}")    
    # Pass global coordinate to robot (x,y)
    # Translate to local frame
    local_coords = transform_to_local_frame(x,y, coordinate_handler.x, coordinate_handler.y, coordinate_handler.euler_angles[2])
    # Error returns the arc tangent in relation to robot local frame. negative sign means right turn, positive sign means left turn
    err = -return_angle(local_coords[0], local_coords[1])
    
    # Debugging
    print(f"Error Angle: {err}")
    #print(f"Distance:{dist_to_wp}")
    angle = 0
    mode = 1
    vel = 0
    # Adjust turn angle according to error
    if 0 > err > -35:
        angle = int(90 + 90*(err/45)) 
        mode = 1
        vel = 50       
    elif 0 < err < 35:
        angle = int(90*(1-(err/-45)))
        mode = 1
        vel = 50
    # If angle is greater than 45 degrees to either side, perform a point turn.
    elif -180 < err < -35:
        mode = 2
        vel = 50
        angle = 0
    elif 180 > err > 35:
        mode = 2
        vel = 50
        angle = 180
    # Set restraints on max values
    if angle < 0:
        angle = 0
    elif angle > 180:
        angle = 180

    coordinate_handler.publish_movement(vel, angle, mode)

def distance(point1, point2):
    return sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)

def generate_path(start, goal, danger_landmarks):
    landmark = danger_landmarks
    # Check whether the new neighboring point is valid. E.g. does it avoid landmarks
    def is_valid(point): 
        return 0 <= point[0] <= 10 and 0 <= point[1] <= 10 and all(distance(point, d) >= coordinate_handler.avoidanceDist for d in landmark.values())

    # Generate new neighbors in pathfinding
    def get_neighbors(point):
        neighbors = []
        # 0.1 increments in all directions
        for dx in [-.1, 0, .1]:
            for dy in [-.1, 0, .1]:
                if dx == 0 and dy == 0:
                    continue
                neighbor = (point[0] + dx, point[1] + dy)
                if is_valid(neighbor):
                    neighbors.append(neighbor)
        return neighbors
    # Round start and goal positions to 2 decimal places (1cm accuracy)
    # Also ensure that coordinates are always positive (Definitely a total hack)
    # Can and should be fixed by using other frames to represent location
    start = (round(start[0], 2)+5, round(start[1]+5, 2))
    goal = (round(goal[0], 2)+5, round(goal[1]+5, 2))
    print(f"Generating path from: {start} to {goal}")

    # Check whether start and goal pose are coincident
    if start == goal:
        return [start]

    # Make A-star variables
    frontier = [(0, start)]
    came_from = {start: None}
    cost_so_far = {start: 0}

    # A-star implementation
    while frontier:
        current_cost, current_node = heappop(frontier)
        dist_to_goal = distance(current_node, goal)
        # Tolerance for successful path found is 10 cm
        if dist_to_goal < 0.1:
            path = []
            while current_node:
                path.append(current_node)
                current_node = came_from[current_node]
                # Remove the +5 increment to each coordinate from earlier (Not ideal)
            return [(element[0]-5,element[1]-5) for element in path[::-1]]

        for next_node in get_neighbors(current_node):
            new_cost = cost_so_far[current_node] + distance(current_node, next_node)
            # Ensure no backtracking OR that the new path "costs" less than others
            if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                cost_so_far[next_node] = new_cost
                priority = new_cost + distance(next_node, goal)
                heappush(frontier, (priority, next_node))
                came_from[next_node] = current_node
    print("No Path Found")
    return None  # No path found

if __name__ == '__main__':
    try:
        coordinate_handler = Handler()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
