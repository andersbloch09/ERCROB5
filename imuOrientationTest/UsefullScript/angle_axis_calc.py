import numpy as np
from scipy.spatial.transform import Rotation

# Your Euler angles (in radians)
roll, pitch, yaw = np.radians([-89, 0, -63])

# Create a rotation object from Euler angles
rotation = Rotation.from_euler('xyz', [roll, pitch, yaw], degrees=False)

#[0.04332941451623777, 0.26972485684182845, 0.4372680928231839, -162.14515612312013, 74.91373118143068, -3.6912042291428007]

# Convert to axis-angle representation
axis_angle = rotation.as_rotvec()

# Extract axis and angle components
axis = axis_angle[:3]
angle = axis_angle[:3]

# Print the result
print("Axis-Angle Representation:")
print("Axis:", axis)
print("Angle (degrees):", np.degrees(angle))
