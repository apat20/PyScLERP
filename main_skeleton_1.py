import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# Importing quaternion functionalities
import func.quaternion_lib as ql

# Import config params
import config.plan_skeleton_1

# Import helper functions
from func.utils import plot_cube
from func.utils import plot_reference_frames
from func.utils import transform_vertices

if __name__ == '__main__':

    # Computing the intermediate configurations using screw linear interpolation for the first constant screw motion:
    [R_array_1, p_array_1, C_array_1, G_array_1, screw_params_1] = ql.sclerp(config.plan_skeleton_1.R_INIT, config.plan_skeleton_1.P_INIT, 
                                                          config.plan_skeleton_1.R_INTER_1, config.plan_skeleton_1.P_INTER_1)

    # Extracting the screw parameters:
    # NOTE: The screw parameters are always computed with respect to the initial configuration given as input to ScLERP.
    theta_1 = screw_params_1[0]
    point_1 = screw_params_1[1]
    unit_vector_1 = screw_params_1[2]
    m_1 = screw_params_1[4]

    print(f"Screw Parameters for the first constant screw motion:\n {screw_params_1}")

    # Computing the intermediate configurations using screw linear interpolation for the second constant screw motion:
    [R_array_2, p_array_2, C_array_2, G_array_2, screw_params_2] = ql.sclerp(config.plan_skeleton_1.R_INTER_1, config.plan_skeleton_1.P_INTER_1, 
                                                          config.plan_skeleton_1.R_INTER_2, config.plan_skeleton_1.P_INTER_2)

    # Extracting the screw parameters:
    # NOTE: The screw parameters are always computed with respect to the initial configuration given as input to ScLERP.
    theta_2 = screw_params_2[0]
    point_2 = screw_params_2[1]
    unit_vector_2 = screw_params_2[2]
    m_2 = screw_params_2[4]

    print(f"Screw Parameters for the second constant screw motion:\n {screw_params_2}")

    # Computing the intermediate configurations using screw linear interpolation for the third constant screw motion:
    [R_array_3, p_array_3, C_array_3, G_array_3, screw_params_3] = ql.sclerp(config.plan_skeleton_1.R_INTER_2, config.plan_skeleton_1.P_INTER_2, 
                                                          config.plan_skeleton_1.R_INTER_3, config.plan_skeleton_1.P_INTER_3)

    # Extracting the screw parameters:
    # NOTE: The screw parameters are always computed with respect to the initial configuration given as input to ScLERP.
    theta_3 = screw_params_3[0]
    point_3 = screw_params_3[1]
    unit_vector_3 = screw_params_3[2]
    m_3 = screw_params_3[4]

    print(f"Screw Parameters for the third constant screw motion:\n {screw_params_3}")

    # Computing the intermediate configurations using screw linear interpolation for the fourth constant screw motion:
    [R_array_4, p_array_4, C_array_4, G_array_4, screw_params_4] = ql.sclerp(config.plan_skeleton_1.R_INTER_3, config.plan_skeleton_1.P_INTER_3, 
                                                          config.plan_skeleton_1.R_INTER_4, config.plan_skeleton_1.P_INTER_4)

    # Extracting the screw parameters:
    # NOTE: The screw parameters are always computed with respect to the initial configuration given as input to ScLERP.
    theta_4 = screw_params_4[0]
    point_4 = screw_params_4[1]
    unit_vector_4 = screw_params_4[2]
    m_4 = screw_params_4[4]

    print(f"Screw Parameters for the fourth constant screw motion:\n {screw_params_4}")

    ########## VISUALIZATION ########## 

    '''PLOT 1:''' 
    # Visualizing the initial and final pose along with all the interpolated poses:
    fig1 = plt.figure()
    ax1 = fig1.add_subplot(projection='3d')
    ax1.grid(False)
    
    # Initial configuration:
    ax1 = plot_reference_frames(config.plan_skeleton_1.R_INIT, np.reshape(config.plan_skeleton_1.P_INIT, [3]), 3, 0.5, ax1)
    
    # Intermediate configuration 1:
    ax1 = plot_reference_frames(config.plan_skeleton_1.R_INTER_1, config.plan_skeleton_1.P_INTER_1, 3, 0.5, ax1)

    # Intermediate configuration 2:
    ax1 = plot_reference_frames(config.plan_skeleton_1.R_INTER_2, config.plan_skeleton_1.P_INTER_2, 3, 0.5, ax1)

    # Intermediate configuration 3:
    ax1 = plot_reference_frames(config.plan_skeleton_1.R_INTER_3, config.plan_skeleton_1.P_INTER_3, 3, 0.5, ax1)

    # Final configuration:
    ax1 = plot_reference_frames(config.plan_skeleton_1.R_INTER_4, config.plan_skeleton_1.P_INTER_4, 3, 0.5, ax1)

    # Intermediate configurations for the first constant screw motion:
    for i in range(R_array_1.shape[2]):
        ax1 = plot_reference_frames(np.reshape(R_array_1[:, :, i], [3,3]), np.reshape(p_array_1[:, :, i], [3]), 2, 0.25, ax1)

    # Intermediate configurations for the second constant screw motion:
    for i in range(R_array_2.shape[2]):
        ax1 = plot_reference_frames(np.reshape(R_array_2[:, :, i], [3,3]), np.reshape(p_array_2[:, :, i], [3]), 2, 0.25, ax1)

    # Intermediate configurations for the second constant screw motion:
    for i in range(R_array_3.shape[2]):
        ax1 = plot_reference_frames(np.reshape(R_array_3[:, :, i], [3,3]), np.reshape(p_array_3[:, :, i], [3]), 2, 0.25, ax1)

    # Intermediate configurations for the second constant screw motion:
    for i in range(R_array_4.shape[2]):
        ax1 = plot_reference_frames(np.reshape(R_array_4[:, :, i], [3,3]), np.reshape(p_array_4[:, :, i], [3]), 2, 0.25, ax1)

    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    ax1.set_xlim(-10, 30)
    ax1.set_ylim(-10, 30)
    ax1.set_zlim(-10, 30)

    '''PLOT 2:'''
    fig2 = plt.figure()
    ax2 = fig2.add_subplot(projection='3d')
    ax2.grid(False)

    # Plot the cuboid at initial pose:
    faces = plot_cube(config.plan_skeleton_1.VERTICES)
    ax2.add_collection3d(Poly3DCollection(faces, linewidths=1, edgecolors='b', alpha=.25))

    # Transform and plot the cuboid at the end of the first constant screw motion:
    transformed_vertices_inter_1 = transform_vertices(config.plan_skeleton_1.G_INTER_1, config.plan_skeleton_1.VERTICES)
    transformed_faces_1 = plot_cube(transformed_vertices_inter_1)
    ax2.add_collection3d(Poly3DCollection(transformed_faces_1, linewidths=1, edgecolors='b', alpha=.25))

    # Transform and plot the cuboid at the end of the second constant screw motion:
    transformed_vertices_inter_2 = transform_vertices(config.plan_skeleton_1.G_INTER_2, config.plan_skeleton_1.VERTICES)
    transformed_faces_2 = plot_cube(transformed_vertices_inter_2)
    ax2.add_collection3d(Poly3DCollection(transformed_faces_2, linewidths=1, edgecolors='b', alpha=.25))

    # Transform and plot the cuboid at the end of the third constant screw motion:
    transformed_vertices_inter_3 = transform_vertices(config.plan_skeleton_1.G_INTER_3, config.plan_skeleton_1.VERTICES)
    transformed_faces_3 = plot_cube(transformed_vertices_inter_3)
    ax2.add_collection3d(Poly3DCollection(transformed_faces_3, linewidths=1, edgecolors='b', alpha=.25))

    # Transform and plot the cuboid at the end of the fourth constant screw motion:
    transformed_vertices_inter_4 = transform_vertices(config.plan_skeleton_1.G_INTER_4, config.plan_skeleton_1.VERTICES)
    transformed_faces_4 = plot_cube(transformed_vertices_inter_4)
    ax2.add_collection3d(Poly3DCollection(transformed_faces_4, linewidths=1, edgecolors='b', alpha=.25))

    # Initial configuration:
    ax2 = plot_reference_frames(config.plan_skeleton_1.R_INIT, np.reshape(config.plan_skeleton_1.P_INIT, [3]), 3, 0.5, ax2)
    
    # Intermediate configuration 1:
    ax2 = plot_reference_frames(config.plan_skeleton_1.R_INTER_2, config.plan_skeleton_1.P_INTER_1, 3, 0.5, ax2)

    # Intermediate configuration 2:
    ax2 = plot_reference_frames(config.plan_skeleton_1.R_INTER_2, config.plan_skeleton_1.P_INTER_2, 3, 0.5, ax2)

    # Intermediate configuration 3:
    ax2 = plot_reference_frames(config.plan_skeleton_1.R_INTER_3, config.plan_skeleton_1.P_INTER_3, 3, 0.5, ax2)

    # Final configuration:
    ax2 = plot_reference_frames(config.plan_skeleton_1.R_INTER_4, config.plan_skeleton_1.P_INTER_4, 3, 0.5, ax2)
    
    # Intermediate configurations for the first constant screw motion:
    for i in range(R_array_1.shape[2]):
        ax2 = plot_reference_frames(np.reshape(R_array_1[:, :, i], [3,3]), np.reshape(p_array_1[:, :, i], [3]), 2, 0.25, ax2)

    # Intermediate configurations for the second constant screw motion:
    for i in range(R_array_2.shape[2]):
        ax2 = plot_reference_frames(np.reshape(R_array_2[:, :, i], [3,3]), np.reshape(p_array_2[:, :, i], [3]), 2, 0.25, ax2)

    # Intermediate configurations for the second constant screw motion:
    for i in range(R_array_3.shape[2]):
        ax2 = plot_reference_frames(np.reshape(R_array_3[:, :, i], [3,3]), np.reshape(p_array_3[:, :, i], [3]), 2, 0.25, ax2)

    # Intermediate configurations for the second constant screw motion:
    for i in range(R_array_4.shape[2]):
        ax2 = plot_reference_frames(np.reshape(R_array_4[:, :, i], [3,3]), np.reshape(p_array_4[:, :, i], [3]), 2, 0.25, ax2)

    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_zlabel('Z')
    ax2.set_xlim(-10, 30)
    ax2.set_ylim(-10, 30)
    ax2.set_zlim(-10, 30)

    '''PLOT 3:'''
    fig3 = plt.figure()
    ax3 = fig3.add_subplot(projection='3d')
    ax3.grid(False)

    # Initial configuration:
    ax3 = plot_reference_frames(config.plan_skeleton_1.R_INIT, np.reshape(config.plan_skeleton_1.P_INIT, [3]), 3, 0.5, ax3)
    
    # Intermediate configuration 1:
    ax3 = plot_reference_frames(config.plan_skeleton_1.R_INTER_1, config.plan_skeleton_1.P_INTER_1, 3, 0.5, ax3)

    # Intermediate configuration 2:
    ax3 = plot_reference_frames(config.plan_skeleton_1.R_INTER_2, config.plan_skeleton_1.P_INTER_2, 3, 0.5, ax3)

    # Intermediate configuration 3:
    ax3 = plot_reference_frames(config.plan_skeleton_1.R_INTER_3, config.plan_skeleton_1.P_INTER_3, 3, 0.5, ax3)

    # Final configuration:
    ax3 = plot_reference_frames(config.plan_skeleton_1.R_INTER_4, config.plan_skeleton_1.P_INTER_4, 3, 0.5, ax3)
    
   # Intermediate configurations for the first constant screw motion:
    for i in range(R_array_1.shape[2]):
        ax3 = plot_reference_frames(np.reshape(R_array_1[:, :, i], [3,3]), np.reshape(p_array_1[:, :, i], [3]), 2, 0.25, ax3)
        # Transform and plot the cuboid at the end of the first constant screw motion:
        transformed_vertices_inter_1 = transform_vertices(G_array_1[:, :, i], config.plan_skeleton_1.VERTICES)
        transformed_faces_1 = plot_cube(transformed_vertices_inter_1)
        ax3.add_collection3d(Poly3DCollection(transformed_faces_1, linewidths=1, facecolors='lightgrey', edgecolors='grey', alpha=.25))

    # Intermediate configurations for the second constant screw motion:
    for i in range(R_array_2.shape[2]):
        ax3 = plot_reference_frames(np.reshape(R_array_2[:, :, i], [3,3]), np.reshape(p_array_2[:, :, i], [3]), 2, 0.25, ax3)
        # Transform and plot the cuboid at the end of the first constant screw motion:
        transformed_vertices_inter_2 = transform_vertices(G_array_2[:, :, i], config.plan_skeleton_1.VERTICES)
        transformed_faces_2 = plot_cube(transformed_vertices_inter_2)
        ax3.add_collection3d(Poly3DCollection(transformed_faces_2, linewidths=1, facecolors='lightgrey', edgecolors='grey', alpha=.25))

    # Intermediate configurations for the second constant screw motion:
    for i in range(R_array_3.shape[2]):
        ax3 = plot_reference_frames(np.reshape(R_array_3[:, :, i], [3,3]), np.reshape(p_array_3[:, :, i], [3]), 2, 0.25, ax3)
        # Transform and plot the cuboid at the end of the first constant screw motion:
        transformed_vertices_inter_3 = transform_vertices(G_array_3[:, :, i], config.plan_skeleton_1.VERTICES)
        transformed_faces_3 = plot_cube(transformed_vertices_inter_3)
        ax3.add_collection3d(Poly3DCollection(transformed_faces_3, linewidths=1, facecolors='lightgrey', edgecolors='grey', alpha=.25))

    # Intermediate configurations for the second constant screw motion:
    for i in range(R_array_4.shape[2]):
        ax3 = plot_reference_frames(np.reshape(R_array_4[:, :, i], [3,3]), np.reshape(p_array_4[:, :, i], [3]), 2, 0.25, ax3)
        # Transform and plot the cuboid at the end of the first constant screw motion:
        transformed_vertices_inter_4 = transform_vertices(G_array_4[:, :, i], config.plan_skeleton_1.VERTICES)
        transformed_faces_4 = plot_cube(transformed_vertices_inter_4)
        ax3.add_collection3d(Poly3DCollection(transformed_faces_4, linewidths=1, facecolors='lightgrey', edgecolors='grey', alpha=.25))

    # Plot the cuboid at initial pose:
    faces = plot_cube(config.plan_skeleton_1.VERTICES)
    ax3.add_collection3d(Poly3DCollection(faces, linewidths=1, edgecolors='b', alpha=.25))

    # Transform and plot the cuboid at the end of the first constant screw motion:
    transformed_vertices_inter_1 = transform_vertices(config.plan_skeleton_1.G_INTER_1, config.plan_skeleton_1.VERTICES)
    transformed_faces_1 = plot_cube(transformed_vertices_inter_1)
    ax3.add_collection3d(Poly3DCollection(transformed_faces_1, linewidths=1, edgecolors='b', alpha=.25))

    # Transform and plot the cuboid at the end of the second constant screw motion:
    transformed_vertices_inter_2 = transform_vertices(config.plan_skeleton_1.G_INTER_2, config.plan_skeleton_1.VERTICES)
    transformed_faces_2 = plot_cube(transformed_vertices_inter_2)
    ax3.add_collection3d(Poly3DCollection(transformed_faces_2, linewidths=1, edgecolors='b', alpha=.25))

    # Transform and plot the cuboid at the end of the third constant screw motion:
    transformed_vertices_inter_3 = transform_vertices(config.plan_skeleton_1.G_INTER_3, config.plan_skeleton_1.VERTICES)
    transformed_faces_3 = plot_cube(transformed_vertices_inter_3)
    ax3.add_collection3d(Poly3DCollection(transformed_faces_3, linewidths=1, edgecolors='b', alpha=.25))

    # Transform and plot the cuboid at the end of the fourth constant screw motion:
    transformed_vertices_inter_4 = transform_vertices(config.plan_skeleton_1.G_INTER_4, config.plan_skeleton_1.VERTICES)
    transformed_faces_4 = plot_cube(transformed_vertices_inter_4)
    ax3.add_collection3d(Poly3DCollection(transformed_faces_4, linewidths=1, edgecolors='b', alpha=.25))

    ax3.set_xlabel('X')
    ax3.set_ylabel('Y')
    ax3.set_zlabel('Z')
    ax3.set_xlim(-10, 30)
    ax3.set_ylim(-10, 30)
    ax3.set_zlim(-10, 30)

    plt.show()