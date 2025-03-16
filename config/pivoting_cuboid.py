# By: Aditya Patankar
import numpy as np

# Dimensions of the cuboid:
X_L = 6
Y_W = 8
Z_H = 10

# Vertices of the cuboid based on the dimensions. All the vertices are 
# expressed with respect to the object's body reference frame.
# NOTE: The object's body reference frame is assumed to be located at its geometric centroid:
VERTICES = np.asarray([np.asarray([X_L/2, -Y_W/2, -Z_H/2]), np.asarray([-X_L/2, -Y_W/2, -Z_H/2]), np.asarray([-X_L/2, Y_W/2, -Z_H/2]), np.asarray([X_L/2, Y_W/2, -Z_H/2]), 
                        np.asarray([X_L/2, -Y_W/2, Z_H/2]), np.asarray([-X_L/2, -Y_W/2, Z_H/2]), np.asarray([-X_L/2, Y_W/2, Z_H/2]), np.asarray([X_L/2, Y_W/2, Z_H/2])])

# Initial configuration:
R_INIT = np.eye(3)
P_INIT = np.zeros([1,3])
G_INIT = np.eye(4)

# Final configuration:
# NOTE: For the pivoting moiton the final configuration is computed based on the dimensions
# of the cuboid:
R_FINAL = np.asarray([[0, 0, 1],
                        [0, 1, 0],
                        [-1, 0, 0]])
if Z_H > X_L:
    P_FINAL = np.asarray([X_L/2 + Z_H/2, 0, -(Z_H/2 - X_L/2)])
elif Z_H < X_L:
    P_FINAL = np.asarray([X_L/2 + Z_H/2, 0, (X_L/2 - Z_H/2)])
else:
    P_FINAL = np.asarray([X_L/2 + Z_H/2, 0, 0])
G_FINAL = np.eye(4)
G_FINAL[0:3, 0:3], G_FINAL[0:3, 3] = R_FINAL, np.reshape(P_FINAL, [3])