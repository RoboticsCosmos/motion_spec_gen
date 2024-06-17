import scipy.spatial
import numpy as np

g = np.array([0.0, 0.0, -9.81])

# Assume that ...
# 1. the base frame is oriented as follows:
#    +x: front of robot
#    +y: left of robot
#    +z: pointing upward
# 2. the arm is pointing "upward" (its base frame aligned with the base frame)
#    then the "Kinova" label is pointing "forward" to the front of the robot
# The rotations are about the moving frame in the following order: YXZ

# 90° to tilt the arm "forward"
# 7.5° to tilt the triangular prism (where both arms are attached to) "forward"
tf_y = scipy.spatial.transform.Rotation.from_euler("y", 97.5, degrees=True)
# 45° to pan to the right arm
tf_rx = scipy.spatial.transform.Rotation.from_euler("x", 45.0, degrees=True)
# 45° to pan to the left arm
tf_lx = scipy.spatial.transform.Rotation.from_euler("x", -45.0, degrees=True)
# 180° to let the "Kinova" label point upward
tf_z = scipy.spatial.transform.Rotation.from_euler("z", 180.0, degrees=True)

tf_r = tf_y * tf_rx * tf_z
tf_l = tf_y * tf_lx * tf_z

tf_r_rpy = scipy.spatial.transform.Rotation.from_euler("xyz", [ 180.0, -82.5, -45.0], degrees=True)
tf_l_rpy = scipy.spatial.transform.Rotation.from_euler("xyz", [-135.0, -80.0, 0.0], degrees=True)

# 6x6 identity matrix
right_identity_mat = np.eye(6)

print("Right arm:")
print(tf_r.as_euler("xyz", degrees=True))
print(tf_r.as_matrix())
print(tf_r_rpy.as_matrix())
print(tf_r.inv().apply(g))

# Initialize an empty 6x6 matrix to store the transformed vectors
transformed_mat = np.zeros((6, 6))

# Apply the transformation to each column
for i in range(6):
    # Extract the i-th column
    column = right_identity_mat[:, i]
    
    # Apply the transformation
    transformed_column1 = tf_r.apply(column[:3])
    transformed_column2 = tf_r.apply(column[3:])
    
    # Store the transformed column
    transformed_mat[:3, i] = transformed_column1
    transformed_mat[3:, i] = transformed_column2

print("Transformed matrix:")
print(np.round(transformed_mat.T, decimals=5))


print()

# print("Left arm:")
# print(tf_l.as_euler("xyz", degrees=True))
# print(tf_l.as_matrix())
# print(tf_l_rpy.as_matrix())
# print(tf_l.inv().apply(g))
