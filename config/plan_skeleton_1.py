# Config file specifying the sequence of constant screw motions the object has to go through. 
# Plan skeleton: {pivot, pickup, transfer, place}
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

'''
    1st Constant Screw Motion: Pivoting
'''
# Final configuration for the pivoting motion:
# NOTE: For the pivoting moiton the final configuration is computed based on the dimensions
# of the cuboid:
R_INTER_1 = np.asarray([[0, 0, 1],
                        [0, 1, 0],
                        [-1, 0, 0]])
if Z_H > X_L:
    P_INTER_1 = np.asarray([X_L/2 + Z_H/2, 0, -(Z_H/2 - X_L/2)])
elif Z_H < X_L:
    P_INTER_1 = np.asarray([X_L/2 + Z_H/2, 0, (X_L/2 - Z_H/2)])
else:
    P_INTER_1 = np.asarray([X_L/2 + Z_H/2, 0, 0])
G_INTER_1 = np.eye(4)
G_INTER_1[0:3, 0:3], G_INTER_1[0:3, 3] = R_INTER_1, np.reshape(P_INTER_1, [3])


'''
    2nd Constant Screw Motion: Pick Up
'''
# Final configuration for the pivoting motion:
# NOTE: For the pivoting moiton the final configuration is computed based on the dimensions
# of the cuboid:
R_INTER_2 = np.asarray([[0, 0, 1],
                        [0, 1, 0],
                        [-1, 0, 0]])

P_INTER_2 = P_INTER_1 + [0,0,20]
G_INTER_2 = np.eye(4)
G_INTER_2[0:3, 0:3], G_INTER_2[0:3, 3] = R_INTER_2, np.reshape(P_INTER_2, [3])


'''
    3rd Constant Screw Motion: Transfer
    NOTE: Here we apply a slight rotation about the X-axis and change the position
'''
# Final configuration for the pivoting motion:
# NOTE: For the pivoting moiton the final configuration is computed based on the dimensions
# of the cuboid:
rot_z = np.asarray([[np.cos(40), -np.sin(40), 0], 
                    [np.sin(40), np.cos(40), 0], 
                    [0, 0, 1]])

R_INTER_3 = np.matmul(rot_z, R_INTER_2)

P_INTER_3 = P_INTER_2 + [10,12,0]
G_INTER_3 = np.eye(4)
G_INTER_3[0:3, 0:3], G_INTER_3[0:3, 3] = R_INTER_3, np.reshape(P_INTER_3, [3])

'''
    4th Constant Screw Motion: Place
    NOTE: Here we place the object back down
'''
# Final configuration for the pivoting motion:
# NOTE: For the pivoting moiton the final configuration is computed based on the dimensions
# of the cuboid:
R_INTER_4 = R_INTER_3

P_INTER_4 = P_INTER_3 + [0,0,-20]
G_INTER_4 = np.eye(4)
G_INTER_4[0:3, 0:3], G_INTER_4[0:3, 3] = R_INTER_4, np.reshape(P_INTER_4, [3])