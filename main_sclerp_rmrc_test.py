# Implementation of ScLERP-based Motion Planning using Resolved Rate Motion Control (RMRC)
# By: Aditya Patankar


import numpy as np
from numpy import linalg as la
import math 
from scipy.spatial.transform import Rotation as R
import func.quaternion_lib as ql

def compute_adjoint(g):
    """
    Compute Adjoint of a 4x4 transformation matrix

    Args:

    Output:

    """
    if g.shape == (4,4):
        
        R, p = g[:3, :3], g[:3, 3]
        p_hat = ql.skew_symmetric(np.reshape(p, [3,1]))
        
        adjoint = np.zeros([6, 6])
        
        adjoint[:3, :3] = R 
        adjoint[:3, 3:6] = p_hat @ R
        adjoint[3:6, 3:6] = R

        return adjoint
    
    else:
        print("Invalid Input Dimensions! Check the dimensions of g.")
    

def compute_twist(w, q):
    """
    Compute twist from unit vector and a point corresponding to the screw axis.

    Args:

    Output:

    """

    if w.shape == (1,3) and q.shape == (1,3):
        eta = np.reshape(np.asarray([-np.cross(w, q), w]), [6,1])
        return eta
    else:
        print("Invalid Input Dimensions! Check the dimensions of w and q.")

def compute_exponential_of_twist(omega, theta, q):
    """
    Exponential of Twist formula 

    Args:

    Output:

    """

    # Printing the shapes of input parameters:
    omega_hat = ql.skew_symmetric(np.reshape(omega, [3,1]))

    exp_omega_hat_theta = np.eye(3) + (omega_hat*np.sin(theta)) + ((omega_hat @ omega_hat)*(1-np.cos(theta)))

    exp_twist_theta = np.eye(4)
    exp_twist_theta[:3, :3], exp_twist_theta[:3, 3] =  exp_omega_hat_theta, np.dot((np.eye(3) - exp_omega_hat_theta), q)
    
    return exp_twist_theta

# --- CRITICAL FIX 1: Vector Scaling instead of Component Clamping ---
def theta_threshold(theta_raw):
    """
    Bound the velocity of the joints by SCALING the vector.
    Preserves direction of motion.
    """
    theta_bounded = theta_raw.copy()
    theta_tolerance = 0.01

    max_val = np.max(np.abs(theta_bounded))
    
    # If the largest component exceeds tolerance, scale the WHOLE vector down
    if max_val > theta_tolerance:
        scale_factor = theta_tolerance / max_val
        theta_bounded = theta_bounded * scale_factor
        
    return theta_bounded

# def theta_threshold(theta_raw):
#     """
#     Bound the values of the joint angles.

#     Args:

#     Output:

#     """
#     num_joints = len(theta_raw)
#     theta_bounded = theta_raw.copy()
#     theta_tolerance = 0.01

#     for i in range(num_joints):
#         if theta_bounded[i] > theta_tolerance:
#             theta_bounded[i] = theta_tolerance
#         elif theta_bounded[i] < -theta_tolerance:
#             theta_bounded[i] = -theta_tolerance
#     return theta_bounded

def compute_error(q1, q2):
    """
    Compute error between two unit dual quaternions representing two transformation

    Args:

    Output: 
    
    """
    # Rotation error:
    rotation_err = min(la.norm(q1[:, 0:4] - q2[:, 0:4]), la.norm(q1[:, 0:4] + q2[:, 0:4]))

    # Extracting the real part:
    quat_r1 = q1[:, 0:4]
    # Extracting the dual part:  
    quat_d1 = np.reshape(q1[:, 4:9], [1,4])
    # Computing the translation vector: 
    p_quat1 = 2*ql.quat_prod(quat_d1, ql.conjugate_quat(quat_r1))
    # The translation vector
    p_vect1 = np.reshape(p_quat1[1:], [3])

    # Extracting the real part:
    quat_r2 = q2[:, 0:4]
    # Extracting the dual part:  
    quat_d2 = np.reshape(q2[:, 4:9], [1,4])
    # Computing the translation vector: 
    p_quat2 = 2*ql.quat_prod(quat_d2, ql.conjugate_quat(quat_r2))
    # The translation vector
    p_vect2 = np.reshape(p_quat2[1:], [3])

    # Position error:
    position_err = la.norm(p_vect1 - p_vect2)
    return rotation_err, position_err

def check_joint_limits_baxter(theta_vec):
    """
    Check the joint limits of Baxter

    Args:

    Output: 
    """

    if theta_vec.shape == (1, 7):
        check_ok = True
        _, num_joints = theta_vec.shape
        up_limits = np.asarray([51, 60, 173, 150, 175, 120, 175])*(np.pi/180)
        low_limits = np.asarray([-141, -123, -173, -3, -175, -90, -175])*(np.pi/180)

        for i in range(num_joints):
            if (theta_vec <= low_limits).any() or (theta_vec >= up_limits).any():
                check_ok = False
                break
    else:
        print("Invalid Input Dimensions! Check the dimensions of theta_vect.")
    return check_ok

def forward_kinematics(gst_0, theta, w, q):
    """
    Forward Kinematics using the Product of Exponential Formula.

    Args:

    Output:

    """
    _ , num_joints = w.shape

    exp_twist_theta = np.zeros([4, 4, num_joints])
    exp_twist_product = np.zeros([4, 4, num_joints])

    for i in range(num_joints):
        exp_twist_theta[:, :, i] = compute_exponential_of_twist(w[:, i], theta[i], q[:, i])
        if i == 0:
            exp_twist_product[:, :, i] = exp_twist_theta[:, :, 0]
        else:
            exp_twist_product[:, :, i] = exp_twist_product[:, :, i-1] @ exp_twist_theta[:, :, i]
    
    gst = exp_twist_product[:, :, num_joints-1] @ gst_0
    
    return gst

def compute_spatial_jacobian(theta, w, q, gst_0):
    """
    Spatial Jacobain of a manipulator

    Args:

    Output: 

    """
    _ , num_joints = w.shape

    twist = np.zeros([6, 1, num_joints])
    exp_twist_theta = np.zeros([4, 4, num_joints])
    
    g1 = np.zeros([4, 4, num_joints])
    adjoint_g1 = np.zeros([6, 6, num_joints])

    gst = np.zeros([4, 4, num_joints])
    spatial_jacobian = np.zeros([6, num_joints])

    for i in range(num_joints):
        twist[:, :, i] = compute_twist(np.reshape(w[:, i], [1, 3]), np.reshape(q[:, i], [1, 3]))
        exp_twist_theta[:, :, i] = compute_exponential_of_twist(w[:, i], theta[i], q[:, i])

    g1[:, :, 0] = np.eye(4)
    spatial_jacobian[:, 0] = np.reshape(twist[:, :, 0], [6])
    gst[:, :, 0] = exp_twist_theta[:, :, 0]

    for i in range(1, num_joints):
        g1[:, :, i] = g1[:, :, i-1] @ exp_twist_theta[:, :, i-1]

    for i in range(num_joints):
        adjoint_g1[:, :, i] = compute_adjoint(g1[:, :, i])
    
    for i in range(1, num_joints):
        spatial_jacobian[:, i] = np.reshape(adjoint_g1[:, :, i] @ twist[:, :, i], [6])
        gst[:, :, i] = gst[:, :, i-1] @ exp_twist_theta[:, :, i]

    for i in range(num_joints):
        gst[:, :, i] = gst[:, :, i] @ gst_0
    
    return gst, spatial_jacobian

def compute_theta_next_step(spatial_jacobian, g_curr, g_next, theta_curr, beta):
    """
    Compute the next joint position using RMRC

    Args:

    Output:

    """
    # Initial pose:
    R_curr, p_curr = g_curr[0:3, 0:3], g_curr[0:3, 3]
    # Convert the rotation matrix into a unit quaternion:
    r_curr = R.from_matrix(R_curr)
    q_curr_scipy = r_curr.as_quat()
    R_curr_quat = np.reshape(np.array([q_curr_scipy[3], q_curr_scipy[0], q_curr_scipy[1], q_curr_scipy[2]]), [4])
    print(R_curr_quat.shape)
    gamma_curr = np.asarray([p_curr[0], p_curr[1], p_curr[2], R_curr_quat[0], R_curr_quat[1], R_curr_quat[2], R_curr_quat[3]])

    # Final pose:
    R_next, p_next = g_next[0:3, 0:3], g_next[0:3, 3]
    # Convert the rotation matrix into a unit quaternion:
    r_next = R.from_matrix(R_next)
    q_next_scipy = r_next.as_quat()
    R_next_quat = np.reshape(np.array([q_next_scipy[3], q_next_scipy[0], q_next_scipy[1], q_next_scipy[2]]), [4])

    # --- CRITICAL FIX 3: Ensure Quaternion Continuity ---
    if np.dot(R_curr_quat, R_next_quat) < 0:
        R_next_quat = -R_next_quat

    gamma_next = np.asarray([p_next[0], p_next[1], p_next[2], R_next_quat[0], R_next_quat[1], R_next_quat[2], R_next_quat[3]])

    p_curr_hat = ql.skew_symmetric(np.reshape(p_curr, [3,1]))

    j1 = np.asarray([[-R_curr_quat[1],  R_curr_quat[0], -R_curr_quat[3],  R_curr_quat[2]], 
                     [-R_curr_quat[2],  R_curr_quat[3],  R_curr_quat[0], -R_curr_quat[1]], 
                     [-R_curr_quat[3], -R_curr_quat[2],  R_curr_quat[1],  R_curr_quat[0]]])
    
    j2 = np.zeros([6, 7])
    j2[:3, :3], j2[3:6, :3] = np.eye(3), np.zeros([3,3])
    j2[:3, 3:7], j2[3:7, 3:7] = (2 * p_curr_hat) @ j1, 2 * j1

    # B = spatial_jacobian.T @ la.solve(spatial_jacobian @ spatial_jacobian.T, j2)

    # --- CRITICAL FIX 2: Use PINV instead of solve for stability ---
    # B = J^T * inv(J * J^T) * j2 is equivalent to pinv(J) * j2 for full rank J
    # Using pinv is safer for numerical stability
    B = np.linalg.pinv(spatial_jacobian) @ j2

    inter_joint_space = beta * B @ (gamma_next -  gamma_curr)

    delta_theta = theta_threshold(inter_joint_space)
    theta_next = theta_curr + delta_theta

    return theta_next

if __name__ == '__main__':
    print("ScLERP-based motion using RMRC!")

    # Initial joint configuration:
    theta_r = np.asarray([-0.4381,0.3422,0.5854,1.3845,-0.1115,1.4144,0.3449])

    # Number of joints/DoF
    num_joints = len(theta_r)

    # Joint origins with respect to Baxter's base reference frame:
    q1 = np.asarray([0.056,0.000,0.011])
    q2 = np.asarray([0.125,0.000,0.281])
    q3 = np.asarray([0.227,0.000,0.281])
    q4 = np.asarray([0.489,0.000,0.213])
    q5 = np.asarray([0.593,-0.001,0.213])
    q6 = np.asarray([0.863,-0.001,0.195])
    q7 = np.asarray([0.979,-0.002,0.195])

    # Frame origins (wrt. arm_mount frame for both the arms)
    qr = np.asarray([q1, q2, q3, q4, q5, q6, q7])

    # Joint axis of the Baxter's arm with respect to the base reference frame:
    w1 = np.asarray([0,0,1])
    w2 = np.asarray([0,1,0]) 
    w3 = np.asarray([1,0,0])

    # Define rotation axes (wrt. arm_mount frame for both the arms)
    w4, w5, w6, w7 = w2, w3, w2, w3
    wr = np.asarray([w1, w2, w3, w4, w5, w6, w7])

    # Defining the end points for the left and the right arms.
    g_st0_right = np.asarray([[0,  0, 1, 1.213],
                              [0,  1, 0, -0.002],
                              [-1, 0, 0, 0.190],
                              [0,  0, 0, 1.0000]])

    # Base transformation of right arm
    g_base_right = np.asarray([[0.7071, 0.7071,  0, 0.025],
                               [-0.7071, 0.7071, 0, -0.220],
                               [0,       0,      1,  0.109],
                               [0,       0,      0,  1.0]])
    
    #  Transform joint axes and origins into base frame
    wr_r = g_base_right[:3, :3] @ wr.T
    qr_r = np.add(g_base_right[:3, :3] @ qr.T, np.reshape(g_base_right[:3, 3], [3, 1]))
    gst0_r = g_base_right @ g_st0_right

    breakpoint()

    g_st_r = forward_kinematics(gst0_r, theta_r, wr_r, qr_r)
    print("Forward Kinematics computed!")

    g_EE_final = np.asarray([[0.9400,    0.2152,   -0.2647,    0.5049],
                             [0.2599,    0.0509,    0.9642,   -0.1059],
                             [0.2210,   -0.9752,   -0.0081,   -0.0102],
                             [0,         0,         0,    1.0000]])
    

    # Convert the rotation matrix into a unit quaternion:
    r_final = R.from_matrix(g_EE_final[:3, :3])
    q_final_scipy = r_final.as_quat()
    R_final_quat = np.reshape(np.array([q_final_scipy[3], q_final_scipy[0], q_final_scipy[1], q_final_scipy[2]]), [1, 4])
    # Convert the position vector into a quaternion:
    p_final_quat = np.reshape(np.append([0], g_EE_final[:3, 3]), [1,4])
    # Unit dual quaternion of the initial pose:
    g_EE_final_unit_dq = np.reshape(np.append(R_final_quat, 1/2*ql.quat_prod(p_final_quat, R_final_quat)), [1,8])

    print(f"g_EE_final_unit_dq:\n {g_EE_final_unit_dq}")
    
    # Spatial Jacobian check: 
    g_spatial, spatial_jac = compute_spatial_jacobian(theta_r, wr_r, qr_r, g_st0_right)

    print("Proceed to next step")


    # Motion Planning for the right arm: 

    beta, counter = 0.1, 0
    goal_not_reached = True
    rotation_tol, position_tol = 0.07, 0.002

    R_curr, p_curr, g_st_r_curr, theta_curr = g_st_r[:3, :3], g_st_r[:3, 3], g_st_r, theta_r

    g_dict = {}
    theta_dict = {}
    spatial_jac_dict = {}
    g_spatial_dict = {}

    while goal_not_reached:
        print(f"COUNTER: {counter}")
        print("\n")

        print(f"R_curr: \n {R_curr}")
        print(f"p_curr: \n {p_curr}")
        print("\n")

        R_array, p_array, C_array, G_array, screw_params = ql.sclerp(R_curr, p_curr, g_EE_final[:3,:3], g_EE_final[:3, 3], tau=0.1)
        g_spatial, spatial_jac = compute_spatial_jacobian(theta_curr, wr_r, qr_r, g_st0_right)

        print(f"spatial_jacobian: \n {spatial_jac}")
        print(spatial_jac.shape)
        print("\n")

        print(f"Next goal: \n {G_array[:, :, 1]}")
        print("\n")

        theta_next = compute_theta_next_step(spatial_jac, g_st_r_curr, G_array[:, :, 1], theta_curr, beta)

        print(f"Theta Next: \n {theta_next}")
        print(theta_next.shape)
        print("\n")

        if not check_joint_limits_baxter(np.reshape(theta_next, [1, 7])):
            print("Joint Limits Violated!")
            break

        g_st_r = forward_kinematics(gst0_r, theta_next, wr_r, qr_r)

        print(f"g_st_r: \n {g_st_r}")
        print("\n")

        # Convert to unit dual quaternion and compute the error:
        r_init = R.from_matrix(g_st_r[:3, :3])
        q_init_scipy = r_init.as_quat() # Returns [x, y, z, w]
        # Manually reorder to [w, x, y, z] for your library
        R_init_quat = np.reshape(np.array([q_init_scipy[3], q_init_scipy[0], q_init_scipy[1], q_init_scipy[2]]), [1, 4])
        # Convert the position vector into a quaternion:
        p_init_quat = np.reshape(np.append([0], g_st_r[:3, 3]), [1,4])
        # Unit dual quaternion of the initial pose:
        g_st_r_unit_dq = np.reshape(np.append(R_init_quat, 1/2*ql.quat_prod(p_init_quat, R_init_quat)), [1,8])

        # g_st_r_unit_dq = np.reshape(np.asarray([0.5109, -0.3750, -0.6022, -0.4856, -0.0092, 0.0921, 0.0539, -0.1476]), [1, 8])

        print(f"g_st_r_unit_dq:\n {g_st_r_unit_dq}")
        print("\n")

        rotation_err, position_err = compute_error(g_st_r_unit_dq, g_EE_final_unit_dq)

        print(f"Rotation error: {rotation_err}")
        print(f"Position error: {position_err}")
        print("\n")
        print("\n############################\n")
        print("\n")

        R_curr, p_curr, g_st_r_curr, theta_curr = g_st_r[:3, :3], g_st_r[:3, 3], g_st_r, theta_next
        counter += 1

        if rotation_err < rotation_tol and position_err < position_tol:
            goal_not_reached = False
            print(f"POSITION GOAL REACHED!")
            print(f"ROTATION GOAL REACHED!")
        elif counter == 1000:
            break







