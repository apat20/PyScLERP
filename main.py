import numpy as np
import matplotlib.pyplot as plt
import quaternion_lib as ql

def skew_symmetric():
    '''
    Function to compute the skew symmetric form of a matrix

    Input Args: 
    
    Returns:
    '''
    return None

def inverse_trans_mat():
    '''
    Function to compute the inverse of a transformation matrix (element of SE(3))

    Input Args: 
    
    Returns:
    '''
    return None

def plot_cube(face_vertices, faces, vertices):
    '''
    Function for plotting a CUBE

    Input Args: 
    
    Returns:
    '''
    # Processing the faces for the cube: 
    # The array 'faces_vertices' is based on the convention 
    faces_vertices = np.asarray([[1,0,3,6], [0,2,5,3], [2,7,4,5], [7,1,6,4], [1,0,2,7], [6,3,5,4]])
    
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
            
'''Function to plot a reference frame:'''
def plot_reference_frames(R, p, scale_value, length_value, ax):
    ax.quiver(p[0], p[1], p[2], scale_value*R[0, 0], scale_value*R[1, 0], scale_value*R[2, 0], color = "r", arrow_length_ratio = length_value)
    ax.quiver(p[0], p[1], p[2], scale_value*R[0, 1], scale_value*R[1, 1], scale_value*R[2, 1], color = "g", arrow_length_ratio = length_value)
    ax.quiver(p[0], p[1], p[2], scale_value*R[0, 2], scale_value*R[1, 2], scale_value*R[2, 2], color = "b", arrow_length_ratio = length_value)
    return ax

if __name__ == '__main__':

    print(f'Inside the main function!')

    # Initial configuration:
    R_init = np.eye(3)
    p_init = np.zeros([1,3])

    # Final configuration:
    R_final = np.asarray([[1, 0, 0],
                          [0, 0, 1],
                          [0, -1, 0]])
    p_final = np.asarray([0, 20, 10])

    [R_array, p_array, C_array, screw_params] = ql.sclerp(R_init, p_init, R_final, p_final)

    # Extracting the screw parameters:
    theta = screw_params[0]
    point = screw_params[1]
    unit_vector = screw_params[2]
    m = screw_params[4]

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
        print(i)
        ax1 = plot_reference_frames(np.reshape(R_array[:, :, i], [3,3]), np.reshape(p_array[:, :, i], [3]), 2, 0.25, ax1)

    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    ax1.set_xlim(-10, 20)
    ax1.set_ylim(-10, 20)
    ax1.set_zlim(-10, 20)

    plt.show()