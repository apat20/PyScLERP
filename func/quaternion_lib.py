# By: Aditya Patankar

import numpy as np
from numpy import linalg as la
import math 
from scipy.spatial.transform import Rotation as R


def skew_symmetric(p):
    """
    Function to compute the skew symmetric matrix of a vector.

    Args: 
        A 3x1 vector
    
    Returns:
        A 3x3 matrix
    """
    if p.shape == (3,1):
        p_hat = np.asarray([[0, float(-p[2]), float(p[1])],
                            [float(p[2]), 0,  float(-p[0])],
                            [float(-p[1]), float(p[0]), 0]]) + 0
        return p_hat
    else:
        print('Invalid input dimensions!')
    

def inverse_trans_mat(mat):
    """
    Function to compute the inverse of a transformation matrix (element of SE(3)).

    Args: 
        A 4x4 matrix, element of SE(3)
    
    Returns:
        A 4x4 matrix, element of SE(3)
    """
    inv_mat = np.eye(4)
    inv_mat[0:3, 0:3] = mat[0:3, 0:3].T
    inv_mat[0:3, 3] = -np.dot(inv_mat[0:3, 0:3], mat[0:3, 3])
    return inv_mat+0.0

def rot_to_axis_angle(rot):
    """
    Function to compute the axis angle representation given a rotation matrix.

    Args:

    Returns:

    """
    return None

def axis_angle_to_rot(axis, angle):
    """
    Function to get a rotation matrix given an axis and a angle

    Args:

    Returns:

    """
    axis = axis/la.norm(axis)

    omega = np.asarray([[0, -axis[2], axis[1]],
                        [axis[2], 0, -axis[0]],
                        [-axis[1], axis[0], 0]])

    R = np.eye(3) + (math.sin(angle)*omega) + ((1 - math.cos(angle))*la.matrix_power(omega, 2))
    return R

def quat_to_rotm(quat):
    """
    Function to convert a quaternion to a rotation matrix.

    Args: 
        A 1x4 unit quaternion (w, x, y, z)
    
    Returns:
        A 3x3 rotation matrix (element of SO(3))
    """
    val = np.reshape(quat, [4])
    scipy_quat = np.array([val[1], val[2], val[3], val[0]]) 
    r = R.from_quat(scipy_quat)
    return r.as_matrix()

def quat_to_tranform(unit_dual_quat):
    """
    Function to convert a unit dual quaternion into a 4x4 transformation matrix (element of SE(3)).

    Args: 
        A 1x8 unit dual quaternion 
    
    Returns:
        A tuple of a 3x3 rotation matrix and a 1x3 position vector
    """
    quat_r = unit_dual_quat[:, 0:4]
    quat_d = unit_dual_quat[:, 4:9]
    rotm = quat_to_rotm(quat_r)
    p_quat = 2*quat_prod(quat_d, conjugate_quat(quat_r))
    p = p_quat[1:]
    return [rotm, p]


def transform_to_quat(transformation_matrix):
    """
    Convert a 4x4 transformation matrix to its corresponding unit dual quaternion

    Args:

    Output:

    """
    return None

def compute_error(q1, q2):
    """
    Compute error between two unit dual quaternions representing two transformation

    Args:

    Output: 
    
    """
    # Rotation error:
    rotation_err = min(la.norm(q1[0:3] - q2[0:3]), la.norm(q1[0:3] + q2[0:3]))

    # Extracting the real part:
    quat_r1 = q1[:, 0:4]
    # Extracting the dual part:  
    quat_d1 = np.reshape(q1[:, 4:9], [1,4])
    # Computing the translation vector: 
    p_quat1 = 2*quat_prod(quat_d1, conjugate_quat(quat_r1))
    # The translation vector
    p_vect1 = np.reshape(p_quat1[1:], [3])

    # Extracting the real part:
    quat_r2 = q2[:, 0:4]
    # Extracting the dual part:  
    quat_d2 = np.reshape(q2[:, 4:9], [1,4])
    # Computing the translation vector: 
    p_quat2 = 2*quat_prod(quat_d2, conjugate_quat(quat_r2))
    # The translation vector
    p_vect2 = np.reshape(p_quat2[1:], [3])

    # Position error:
    position_err = la.norm(p_vect1 - p_vect2)
    return rotation_err, position_err

def conjugate_quat(quat):
    """
    Function to compute the conjugate of a quaternion.

    Args: 
        A 1x4 dimension of quaternion.
    
    Returns:
        A 1x4 quaternion conjugate.
    """
    quat = np.reshape(quat, [1,4])
    q_0 = quat[:, 0]
    q_r = np.multiply(quat[:, 1:4], -1)
    return np.reshape(np.append(q_0, q_r+0.0), [1,4])


def conjugate_dual_quat(dual_quat):
    """
    Function to compute conjugate of a unit dual quaternion.

    Args: 
        A 1x8 unit dual quaternion.
    
    Returns:
        A 1x8 conjugate of a unit dual quaternion.
    """
    if dual_quat.shape[1] == 8:
        dual_quat_star = np.asarray([dual_quat[:, 0], -dual_quat[:, 1], -dual_quat[:, 2], -dual_quat[:, 3],
                                dual_quat[:, 4], -dual_quat[:, 5], -dual_quat[:, 6], -dual_quat[:, 7]]) + 0
        return np.reshape(dual_quat_star, [1, 8])
    else:
        print("Incorrect input dimensions!")


def quat_prod(quat_1, quat_2):
    """
    Function to compute product of two unit quaternions.
    
    Args: 
        Two 1x4 unit quaternions.
    
    Returns:
        A 1x4 unit quaternion.
    """
    if quat_1.shape[1] and quat_2.shape[1] == 4:
        a_0 = quat_1[:, 0]
        b_0 = quat_2[:, 0]
        a = quat_1[:, 1:4]
        b = quat_2[:, 1:4]
        scalar = (a_0*b_0) - np.dot(a, np.transpose(b))
        vector = (a_0*b) + (b_0*a) + np.cross(a, b)
        prod = np.append(scalar, vector)
    else:
        print(f'Incorrect input dimension!')
    return prod + 0


def dual_quat_prod(quat_1, quat_2):
    """
    Function to compute dual quaternion product.

    Args: 
        Two 1x8 unit dual quaternions.

    Returns:
        A 1x8 unit dual quaternion.
    """
    p = quat_1[:, 0:4]
    q = quat_1[:, 4:9]
    u = quat_2[:, 0:4]
    v = quat_2[:, 4:9]
    prod = np.append(quat_prod(p, u), (quat_prod(q, u) + quat_prod(p, v)))
    return prod + 0


def get_screw_params(g):
    """
    Function to compute the screw parameters given. 
    """
    pass


def get_screw_params(g_init, g_final):
    """
    Function to compute the screw parameters given two poses.

    Args: 
        Two 4x4 poses in SE(3).

    Returns:
        A list containing the following parameters:
            theta: Magnitude of screw motion.
            point: A point on the screw axis.
            u: Direction of the screw axis.
            pitch: Pitch of the screw.
            m: Moment of the screw.
            d: Magnitude of displacement along the screw.
            p_vect: Translation vector.
    """
    # Initial pose:
    R_init, p_init = g_init[0:3, 0:3], g_init[0:3, 3]
    
    # Convert the rotation matrix into a unit quaternion:
    r_init = R.from_matrix(R_init)
    R_init_quat = np.reshape(r_init.as_quat(scalar_first=True), [1,4])
    # Convert the position vector into a quaternion:
    p_init_quat = np.reshape(np.append([0], p_init), [1,4])
    # Unit dual quaternion of the initial pose:
    g_init_unit_quat = np.reshape(np.append(R_init_quat, 1/2*quat_prod(p_init_quat, R_init_quat)), [1,8])
    
    # Final pose:
    R_final, p_final = g_final[0:3, 0:3], g_final[0:3, 3]
    
    # Convert the rotation matrix into a unit quaternion:
    r_final = R.from_matrix(R_final)
    R_final_quat = np.reshape(r_final.as_quat(scalar_first=True), [1,4])
    # Convert the position vector into a quaternion:
    p_final_quat = np.reshape(np.append([0], p_final), [1,4])
    # Unit dual quaternion of the initial pose:
    g_final_unit_quat = np.reshape(np.append(R_final_quat, 1/2*quat_prod(p_final_quat, R_final_quat)), [1,8])
    
    # Compute the unit dual quaternion corresponding to the relative transformation:
    D = np.reshape(dual_quat_prod(conjugate_dual_quat(g_init_unit_quat), g_final_unit_quat), [1,8])
    
    # Computing the screw parameters:
    return get_screw_params_dual_quat(D, g_init, g_final)


def get_screw_params_dual_quat(unit_dual_quat):
    """
    Function to compute screw parameters given a unit dual quaternion corresponding to a relative transformation.
    
    Args: 
        A 1x8 unit dual quaternion

    Returns:
        A list containing the following parameters:
            theta: Magnitude of screw motion
            point: A point on the screw axis
            u: Direction of the screw axis
            pitch: Pitch of the screw
            m: Moment of the screw
            d: Magnitude of displacement along the screw
            p_vect: Translation vector
    """

    # Extracting the real part of the unit dual quaternion:
    quat_r = unit_dual_quat[:, 0:4]
    scalar_quat_r = quat_r[:, 0]
    vector_quat_r = quat_r[:, 1:4]

    # Extracting the dual part:
    quat_d = np.reshape(unit_dual_quat[:, 4:9], [1,4])
    # Computing the translation vector: 
    p_quat = 2*quat_prod(quat_d, conjugate_quat(quat_r))
    # The translation vector
    p_vect = np.reshape(p_quat[1:], [3])

    # PURE TRANSLATION:
    if la.norm(vector_quat_r) <= 1e-12:
        u = np.reshape(p_vect/la.norm(p_vect), [3])
        theta = 0
        d = np.dot(p_vect, u)
        m = np.asarray([0, 0, 0])
        point = np.asarray([0, 0, 0])
        pitch = math.inf
    # GENERAL CONSTANT SCREW MOTION:
    else:
        u = np.reshape(np.divide(vector_quat_r, la.norm(vector_quat_r)), [3])
        theta = 2*np.arctan2(la.norm(vector_quat_r), scalar_quat_r)
        # Keeping theta between 0 to pi
        if theta > math.pi:
            theta = 2*math.pi - theta
            u = -u

        d = np.dot(p_vect, u)
        m = 1/2*(np.cross(p_vect, u) + (p_vect - d*u)*(1/(np.tan(theta/2))))
        point = np.cross(u, m)
        pitch = d/theta
    
    return [theta+0.0, point+0.0, u+0.0, pitch+0.0, p_vect+0.0, m+0.0, d+0.0]


def sclerp(R_init, p_init, R_final, p_final, tau):
    """
    Function to perform screw linear interpolation given unit dual quaternion representation of two poses in SE(3).

    Args: 
    
    Returns:
    
    """
    # Initial pose:
    g_init = np.eye(4,4)
    g_init[0:3, 0:3] = R_init
    g_init[0:3, 3] = p_init

    # Convert the rotation matrix into a unit quaternion:
    
    # r_init = R.from_matrix(R_init)
    # R_init_quat = np.reshape(r_init.as_quat(scalar_first=True), [1,4])

    # FIX: SciPy [x,y,z,w] -> [w,x,y,z]
    r_init = R.from_matrix(R_init)
    q = r_init.as_quat()
    R_init_quat = np.array([[q[3], q[0], q[1], q[2]]])

    # Convert the position vector into a quaternion:
    p_init_quat = np.reshape(np.append([0], p_init), [1,4])
    # Unit dual quaternion of the initial pose:
    g_init_unit_quat = np.reshape(np.append(R_init_quat, 1/2*quat_prod(p_init_quat, R_init_quat)), [1,8])

    # Final pose:
    g_final = np.eye(4,4)
    g_final[0:3, 0:3] = R_final
    g_final[0:3, 3] = p_final

    # Convert the rotation matrix into a unit quaternion:
    
    # r_final = R.from_matrix(R_final)
    # R_final_quat = np.reshape(r_final.as_quat(scalar_first=True), [1,4])

    # FIX: SciPy [x,y,z,w] -> [w,x,y,z]
    r_final = R.from_matrix(R_final)
    q = r_final.as_quat()
    R_final_quat = np.array([[q[3], q[0], q[1], q[2]]])

    # Convert the position vector into a quaternion:
    p_final_quat = np.reshape(np.append([0], p_final), [1,4])
    # Unit dual quaternion of the initial pose:
    g_final_unit_quat = np.reshape(np.append(R_final_quat, 1/2*quat_prod(p_final_quat, R_final_quat)), [1,8])

    # Compute relative Dual Quaternion
    # Robustness: Check dot product to avoid taking the long way around
    # (Optional but recommended for Sclerp)
    if np.dot(g_init_unit_quat.flatten(), g_final_unit_quat.flatten()) < 0:
        g_final_unit_quat = -g_final_unit_quat

    # Compute the unit dual quaternion corresponding to the relative transformation:
    D = np.reshape(dual_quat_prod(conjugate_dual_quat(g_init_unit_quat), g_final_unit_quat), [1,8])
    print(f'Computing the screw parameters!')

    # Computing the screw parameters:
    screw_params =  get_screw_params_dual_quat(D)
    print(f'Performing screw interpolation')

    # tau is the interpolation parameter:
    tau_vect = np.arange(0, 1.1, tau)

    # Magnitude:
    theta = screw_params[0]
    # Point on the screw axis:
    point = screw_params[1]
    # Unite vector corresponding to the screw axis
    unit_vector = screw_params[2]
    # Screw pitch
    pitch = screw_params[3]
    # Moment
    m = screw_params[5]
    # Displacement along the axis
    d = screw_params[6]

    # Initializing empty multidimensional arrays to save the computed intermediate interpolated poses:
    R_array, p_array = np.zeros([3,3,len(tau_vect)]), np.zeros([3,1,len(tau_vect)])
    C_dual_quat_array, G_array = np.zeros([8, len(tau_vect)]), np.zeros([4,4,len(tau_vect)])

    for i in range(len(tau_vect)):
        # Computing the real and the dual parts of the unit dual quaternion corresponding to the intermediate 
        # configurations computed using the interpolation scheme.
        # Equations (39) and (44) from Yan-Bin Jia's notes have been used for this purpose
        D_r = np.reshape(np.append(np.cos((tau_vect[i]*theta)/2), unit_vector*np.sin((tau_vect[i]*theta)/2)), [1, 4])
        D_r[np.isnan(D_r)] = 0
        D_d = np.reshape(np.append(-(tau_vect[i]*d)/2*np.sin((tau_vect[i]*theta)/2), unit_vector*(tau_vect[i]*d)/2*np.cos((tau_vect[i]*theta)/2) + np.sin((tau_vect[i]*theta)/2)*m), [1, 4])
        D_d[np.isnan(D_d)] = 0
        C_dual_quat_array[:, i] = dual_quat_prod(g_init_unit_quat, np.reshape(np.append(D_r, D_d), [1, 8]))
        
        # Computing the rotation matrix and position vector for a particular configuration from its corresponding 
        # unit dual quaternion representation:
        g = quat_to_tranform(np.reshape(C_dual_quat_array[:, i], [1, 8]))
        G_array[0:3, 0:3, i], G_array[0:3, 3, i] = np.reshape(g[0], [3,3]), np.reshape(g[1], [3])
        R_array[:, :, i], p_array[:, :, i] = np.reshape(g[0], [3,3]), np.reshape(g[1], [3,1])
        

    return [R_array, p_array, C_dual_quat_array, G_array, screw_params]

def sclerp_tau(R_init, p_init, R_final, p_final, tau):
    """
    Function to perform screw linear interpolation given unit dual quaternion representation of two poses in SE(3).

    Args: 
    
    Returns:
    
    """
    # Initial pose:
    g_init = np.eye(4,4)
    g_init[0:3, 0:3] = R_init
    g_init[0:3, 3] = p_init

    # Convert the rotation matrix into a unit quaternion:
    r_init = R.from_matrix(R_init)
    R_init_quat = np.reshape(r_init.as_quat(scalar_first=True), [1,4])
    # Convert the position vector into a quaternion:
    p_init_quat = np.reshape(np.append([0], p_init), [1,4])
    # Unit dual quaternion of the initial pose:
    g_init_unit_quat = np.reshape(np.append(R_init_quat, 1/2*quat_prod(p_init_quat, R_init_quat)), [1,8])

    # Final pose:
    g_final = np.eye(4,4)
    g_final[0:3, 0:3] = R_final
    g_final[0:3, 3] = p_final

    # Convert the rotation matrix into a unit quaternion:
    r_final = R.from_matrix(R_final)
    R_final_quat = np.reshape(r_final.as_quat(scalar_first=True), [1,4])
    # Convert the position vector into a quaternion:
    p_final_quat = np.reshape(np.append([0], p_final), [1,4])
    # Unit dual quaternion of the initial pose:
    g_final_unit_quat = np.reshape(np.append(R_final_quat, 1/2*quat_prod(p_final_quat, R_final_quat)), [1,8])

    # Compute the unit dual quaternion corresponding to the relative transformation:
    D = np.reshape(dual_quat_prod(conjugate_dual_quat(g_init_unit_quat), g_final_unit_quat), [1,8])
    print(f'Computing the screw parameters!')

    # Computing the screw parameters:
    screw_params =  get_screw_params_dual_quat(D)
    print(f'Performing screw interpolation')

    # tau is the interpolation parameter:
    tau_vect = np.asarray([tau])

    # Magnitude:
    theta = screw_params[0]
    # Point on the screw axis:
    point = screw_params[1]
    # Unite vector corresponding to the screw axis
    unit_vector = screw_params[2]
    # Screw pitch
    pitch = screw_params[3]
    # Moment
    m = screw_params[5]
    # Displacement along the axis
    d = screw_params[6]

    # Initializing empty multidimensional arrays to save the computed intermediate interpolated poses:
    R_array, p_array = np.zeros([3,3,len(tau_vect)]), np.zeros([3,1,len(tau_vect)])
    C_dual_quat_array, G_array = np.zeros([8, len(tau_vect)]), np.zeros([4,4,len(tau_vect)])

    for i in range(len(tau_vect)):
        # Computing the real and the dual parts of the unit dual quaternion corresponding to the intermediate 
        # configurations computed using the interpolation scheme.
        # Equations (39) and (44) from Yan-Bin Jia's notes have been used for this purpose
        D_r = np.reshape(np.append(np.cos((tau_vect[i]*theta)/2), unit_vector*np.sin((tau_vect[i]*theta)/2)), [1, 4])
        D_r[np.isnan(D_r)] = 0
        D_d = np.reshape(np.append(-(tau_vect[i]*d)/2*np.sin((tau_vect[i]*theta)/2), unit_vector*(tau_vect[i]*d)/2*np.cos((tau_vect[i]*theta)/2) + np.sin((tau_vect[i]*theta)/2)*m), [1, 4])
        D_d[np.isnan(D_d)] = 0
        C_dual_quat_array[:, i] = dual_quat_prod(g_init_unit_quat, np.reshape(np.append(D_r, D_d), [1, 8]))
        
        # Computing the rotation matrix and position vector for a particular configuration from its corresponding 
        # unit dual quaternion representation:
        g = quat_to_tranform(np.reshape(C_dual_quat_array[:, i], [1, 8]))
        G_array[0:3, 0:3, i], G_array[0:3, 3, i] = np.reshape(g[0], [3,3]), np.reshape(g[1], [3])
        R_array[:, :, i], p_array[:, :, i] = np.reshape(g[0], [3,3]), np.reshape(g[1], [3,1])
        

    return [R_array, p_array, C_dual_quat_array, G_array, screw_params]









