import numpy as np
from scipy.spatial.transform import Rotation

# Your Euler angles (in radians)
roll, pitch, yaw = np.radians([15, 0, 0])

# Create a rotation object from Euler angles
rotation = Rotation.from_euler('xyz', [roll, pitch, yaw], degrees=False)

# Convert to axis-angle representation
axis_angle = rotation.as_rotvec()

# Extract axis and angle components
axis = axis_angle[:3]
angle = axis_angle[:3]

# Print the result
print("Axis-Angle Representation:")
print("Axis:", axis)
print("Angle (degrees):", np.degrees(angle))
