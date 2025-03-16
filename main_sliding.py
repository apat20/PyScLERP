import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import quaternion_lib as ql

# Import config params
import config.pivoting_cuboid

# Import helper functions
import func._helper_functions as hlp

if __name__ == '__main__':

    # Screw linear interpolation between a pair of initial and final pose:
    [R_array, p_array, C_array, screw_params] = ql.sclerp(config.sliding_cuboid.R_INIT, config.sliding_cuboid.P_INIT, 
                                                          config.sliding_cuboid.R_FINAL, config.sliding_cuboid.P_FINAL)

    # Extracting the screw parameters:
    theta = screw_params[0]
    point = screw_params[1]
    unit_vector = screw_params[2]
    m = screw_params[4]

    print(f"Screw Parameters:\n {screw_params}")

    ########## VISUALIZATION ########## 

    '''PLOT 1:''' 
    # Visualizing the initial and final pose along with all the interpolated poses:
    fig1 = plt.figure()
    ax1 = fig1.add_subplot(projection='3d')
    ax1.grid(False)

    # Plot the screw axis and the point:
    ax1.scatter(point[0], point[1], point[2], marker = '*', s = 100, color = 'r')
    ax1.quiver(point[0], point[1], point[2], 2*unit_vector[0], 2*unit_vector[1], 2*unit_vector[2], color = "r", arrow_length_ratio = 0.55)

    # Initial configuration:
    ax1 = hlp.plot_reference_frames(config.sliding_cuboid.R_INIT, np.reshape(config.sliding_cuboid.P_INIT, [3]), 3, 0.5, ax1)

    # Final configuration:
    ax1 = hlp.plot_reference_frames(config.sliding_cuboid.R_FINAL, config.sliding_cuboid.P_FINAL, 3, 0.5, ax1)

    # Intermediate configurations:
    for i in range(R_array.shape[2]):
        ax1 = hlp.plot_reference_frames(np.reshape(R_array[:, :, i], [3,3]), np.reshape(p_array[:, :, i], [3]), 2, 0.25, ax1)

    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    ax1.set_xlim(-10, 20)
    ax1.set_ylim(-10, 20)
    ax1.set_zlim(-10, 20)

    '''PLOT 2:''' 
    fig2 = plt.figure()
    ax2 = fig2.add_subplot(projection='3d')
    ax2.grid(False)

    # Plot the cuboid at initial pose:
    faces = hlp.plot_cube(config.sliding_cuboid.VERTICES)
    ax2.add_collection3d(Poly3DCollection(faces, linewidths=1, edgecolors='b', alpha=.25))
    
    # Transform and plot the cuboid at the final pose:
    transformed_vertices_final = hlp.transform_vertices(config.sliding_cuboid.G_FINAL, config.sliding_cuboid.VERTICES)
    transformed_faces = hlp.plot_cube(transformed_vertices_final)
    ax2.add_collection3d(Poly3DCollection(transformed_faces, linewidths=1, edgecolors='b', alpha=.25))

    # Plot the screw axis and the point:
    ax2.scatter(point[0], point[1], point[2], marker = '*', s = 100, color = 'r')
    ax2.quiver(point[0], point[1], point[2], 2*unit_vector[0], 2*unit_vector[1], 2*unit_vector[2], color = "r", arrow_length_ratio = 0.55)

    # Initial configuration:
    ax2 = hlp.plot_reference_frames(config.sliding_cuboid.R_INIT, np.reshape(config.sliding_cuboid.P_INIT, [3]), 3, 0.5, ax2)

    # Final configuration:
    ax2 = hlp.plot_reference_frames(config.sliding_cuboid.R_FINAL, config.sliding_cuboid.P_FINAL, 3, 0.5, ax2)

    # Intermediate configurations:
    for i in range(R_array.shape[2]):
        ax2 = hlp.plot_reference_frames(np.reshape(R_array[:, :, i], [3,3]), np.reshape(p_array[:, :, i], [3]), 2, 0.25, ax2)

    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_zlabel('Z')
    ax2.set_xlim(-10, 20)
    ax2.set_ylim(-10, 20)
    ax2.set_zlim(-10, 20)

    plt.show()