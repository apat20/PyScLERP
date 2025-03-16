import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import quaternion_lib as ql

def plot_cube(vertices):
    '''
    Function for plotting a CUBE

    Input Args: 
    
    Returns:
    '''
    # Processing the faces for the cube: 
    # The array 'faces_vertices' is based on a predefined convention
    faces_vertices = np.asarray([[0,4,7,3], [0,1,5,4], [1,2,6,5], [3,7,6,2], [4,5,6,7], [0,1,2,3]])
    
    # Initialize a list of vertex coordinates for each face
    faces = []
    faces.append(np.zeros([4,3]))
    faces.append(np.zeros([4,3]))
    faces.append(np.zeros([4,3]))
    faces.append(np.zeros([4,3]))
    faces.append(np.zeros([4,3]))
    faces.append(np.zeros([4,3]))
    
    for i, v in enumerate(faces_vertices):
        for j in range(faces_vertices.shape[1]):
            faces[i][j, 0] = vertices[faces_vertices[i,j],0]
            faces[i][j, 1] = vertices[faces_vertices[i,j],1]
            faces[i][j, 2] = vertices[faces_vertices[i,j],2]

    return faces

def transform_vertices(g, vertices):
    '''
    Function to transform the vertices of a cuboid and express them with respect to the current pose.
    # NOTE: This function assumes that the pose in which the vertices are expressed initially is identity
    
    Input Args:

    Returns:

    '''
    transformed_vertices = np.zeros([vertices.shape[0], vertices.shape[1]])
    for i, vertex in enumerate(vertices):
        hom_vertex = np.matmul(g, np.reshape(np.append(vertex, 1), [4,1]))
        transformed_vertices[i,:] = np.reshape(hom_vertex[0:3, :], [3])
    return transformed_vertices
            
def plot_reference_frames(R, p, scale_value, length_value, ax):
    '''Function to plot a reference frame:'''
    ax.quiver(p[0], p[1], p[2], scale_value*R[0, 0], scale_value*R[1, 0], scale_value*R[2, 0], color = "r", arrow_length_ratio = length_value)
    ax.quiver(p[0], p[1], p[2], scale_value*R[0, 1], scale_value*R[1, 1], scale_value*R[2, 1], color = "g", arrow_length_ratio = length_value)
    ax.quiver(p[0], p[1], p[2], scale_value*R[0, 2], scale_value*R[1, 2], scale_value*R[2, 2], color = "b", arrow_length_ratio = length_value)
    return ax

if __name__ == '__main__':

    # Dimensions of the cuboid:
    X_L = 6
    Y_W = 8
    Z_H = 10

    # Vertices of the cuboid based on the dimensions. All the vertices are 
    # expressed with respect to the object's body reference frame.
    # NOTE: The object's body reference frame is assumed to be located at its geometric centroid:
    vertices = np.asarray([np.asarray([X_L/2, -Y_W/2, -Z_H/2]), np.asarray([-X_L/2, -Y_W/2, -Z_H/2]), np.asarray([-X_L/2, Y_W/2, -Z_H/2]), np.asarray([X_L/2, Y_W/2, -Z_H/2]), 
                           np.asarray([X_L/2, -Y_W/2, Z_H/2]), np.asarray([-X_L/2, -Y_W/2, Z_H/2]), np.asarray([-X_L/2, Y_W/2, Z_H/2]), np.asarray([X_L/2, Y_W/2, Z_H/2])])

    # Initial configuration:
    R_init = np.eye(3)
    p_init = np.zeros([1,3])
    g_init = np.eye(4)

    # Final configuration:
    # NOTE: For the pivoting moiton the final configuration is computed based on the dimensions
    # of the cuboid:
    R_final = np.asarray([[0, 0, 1],
                          [0, 1, 0],
                          [-1, 0, 0]])
    p_final = np.asarray([X_L/2 + Z_H/2, 0, -(Z_H/2 - X_L/2)])
    g_final = np.eye(4)
    g_final[0:3, 0:3], g_final[0:3, 3] = R_final, np.reshape(p_final, [3])

    [R_array, p_array, C_array, screw_params] = ql.sclerp(R_init, p_init, R_final, p_final)

    # Extracting the screw parameters:
    theta = screw_params[0]
    point = screw_params[1]
    unit_vector = screw_params[2]
    m = screw_params[4]

    print(f"Screw Parameters: {screw_params}")

    ### VISUALIZATION:

    # Visualizing the initial and final pose along with all the interpolated poses:
    fig1 = plt.figure()
    ax1 = fig1.add_subplot(projection='3d')
    ax1.grid(False)
    # Plot the screw axis and the point:
    ax1.scatter(point[0], point[1], point[2], marker = '*', s = 100, color = 'r')
    ax1.quiver(point[0], point[1], point[2], 2*unit_vector[0], 2*unit_vector[1], 2*unit_vector[2], color = "r", arrow_length_ratio = 0.55)

    # Initial configuration:
    ax1 = plot_reference_frames(R_init, np.reshape(p_init, [3]), 3, 0.5, ax1)

    # Final configuration:
    ax1 = plot_reference_frames(R_final, p_final, 3, 0.5, ax1)

    # Intermediate configurations:
    for i in range(R_array.shape[2]):
        ax1 = plot_reference_frames(np.reshape(R_array[:, :, i], [3,3]), np.reshape(p_array[:, :, i], [3]), 2, 0.25, ax1)

    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    ax1.set_xlim(-10, 10)
    ax1.set_ylim(-10, 10)
    ax1.set_zlim(-10, 10)

    fig2 = plt.figure()
    ax2 = fig2.add_subplot(projection='3d')
    ax2.grid(False)

    # Plot the cuboid at initial pose:
    faces = plot_cube(vertices)
    ax2.add_collection3d(Poly3DCollection(faces, linewidths=1, edgecolors='b', alpha=.25))
    
    # Transform and plot the cuboid at the final pose:
    transformed_vertices_final = transform_vertices(g_final, vertices)
    transformed_faces = plot_cube(transformed_vertices_final)
    ax2.add_collection3d(Poly3DCollection(transformed_faces, linewidths=1, edgecolors='b', alpha=.25))

    # Plot the screw axis and the point:
    ax2.scatter(point[0], point[1], point[2], marker = '*', s = 100, color = 'r')
    ax2.quiver(point[0], point[1], point[2], 2*unit_vector[0], 2*unit_vector[1], 2*unit_vector[2], color = "r", arrow_length_ratio = 0.55)

    # Initial configuration:
    ax2 = plot_reference_frames(R_init, np.reshape(p_init, [3]), 3, 0.5, ax2)

    # Final configuration:
    ax2 = plot_reference_frames(R_final, p_final, 3, 0.5, ax2)

    # Intermediate configurations:
    for i in range(R_array.shape[2]):
        ax2 = plot_reference_frames(np.reshape(R_array[:, :, i], [3,3]), np.reshape(p_array[:, :, i], [3]), 2, 0.25, ax2)

    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_zlabel('Z')
    ax2.set_xlim(-10, 10)
    ax2.set_ylim(-10, 10)
    ax2.set_zlim(-10, 10)


    plt.show()