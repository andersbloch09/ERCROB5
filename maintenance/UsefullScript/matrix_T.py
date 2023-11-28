import rtde_control
import rtde_receive
from math import pi
import numpy as np 
from scipy.spatial.transform import Rotation

IP = "192.168.1.102"

rtde_c = rtde_control.RTDEControlInterface(IP)
rtde_r = rtde_receive.RTDEReceiveInterface(IP)

def euler_to_angle_axis(roll, pitch, yaw):
        # Your Euler angles (in radians)

    # Create a rotation object from Euler angles
    rotation = Rotation.from_euler('xyz', [roll, pitch, yaw], degrees=True)

    # Convert to axis-angle representation
    axis_angle = rotation.as_rotvec()

    # Extract axis and angle components
    angle = axis_angle[:3]

    # Print the result
    print("Axis-Angle Representation:")
    print("Angle (degrees):", np.degrees(angle))

    return np.degrees(angle[0]), np.degrees(angle[1]), np.degrees(angle[2]) 

def rad2deg(list):
    newlist = []
    for j in range(3):
        newlist.append(list[j])
    for i in range(3, 6):
        newlist.append(list[i] * (180/pi))

    return newlist

tcp_pose = rtde_r.getActualTCPPose()

tcp_pose = rad2deg(tcp_pose)

print(tcp_pose)


def create_transformation_matrix(x, y, z, roll, pitch, yaw):
    """
    Create a 4x4 transformation matrix for a given position and orientation.
    """
   # Create a rotation matrix from Euler angles with 'zyx' order
    rotation_matrix = Rotation.from_euler('xyz', [roll, pitch, yaw], degrees=False).as_matrix()
    #angles = np.array([roll,pitch,yaw])
    #rotation_matrix = Rotation.from_rotvec(angles).as_matrix()
    #rotation_matrix = Rotation.from_rotvec(angles).as_matrix()

    # Create a 4x4 transformation matrix
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = rotation_matrix
    transformation_matrix[:3, 3] = [x, y, z]
    
    print(transformation_matrix)

    return transformation_matrix

# Example: Desired position and orientation
x,y,z = [0.25, 0.25, 0.25]

roll = np.radians(-84) # Convert degrees to radians
pitch = np.radians(35)
yaw = np.radians(-35)

# Create the transformation matrix
transform_matrix = create_transformation_matrix(x, y, z, roll, pitch, yaw)


# Extract rotation component
rotation_matrix = transform_matrix[:3, :3]
rotation = Rotation.from_matrix(rotation_matrix)

# Convert to Euler angles
euler_angles = rotation.as_euler('xyz', degrees=True)  # You can choose the order of rotations 'xyz' as needed

#Print the result
#print("Euler Angles (XYZ):", euler_angles)


#print("Transformation Matrix:")
#print(transform_matrix)


x_rotation, y_rotation, z_rotation = euler_to_angle_axis(0, 0, 0)

print(x_rotation, y_rotation, z_rotation)

p = np.array([[0], [0], [0], [1]])


dot_product = np.dot(transform_matrix, p)

print("Dot product", dot_product)

velocity = 3
acceleration = 1.4
blend_1 = 0.0

def deg2rad(list):
    newlist = []
    for j in range(3):
        newlist.append(list[j])
    for i in range(3, 6):
        newlist.append(list[i]*pi/180)

    return newlist

euler_angles[0] = euler_angles[0] + x_rotation 
euler_angles[1] = euler_angles[1] + y_rotation
euler_angles[2] = euler_angles[2] + z_rotation 

print("Euler angles", euler_angles)

path_pose1 = [dot_product[0], dot_product[1], dot_product[2], euler_angles[0], euler_angles[1], euler_angles[2]]

path_pose1 = deg2rad(path_pose1)

#path_pose1 = rtde_c.getInverseKinematics(path_pose1)

path_pose1.extend([velocity, acceleration, blend_1])

path = [path_pose1]

# Send a linear path with blending in between - (currently uses separate script)
rtde_c.moveL(path)
rtde_c.stopScript()