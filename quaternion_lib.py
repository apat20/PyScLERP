import numpy as np
from numpy import linalg as la
import math 
from scipy.spatial.transform import Rotation as R

# NOTE: Quaternions -q and q represent the same rotation? Need to check this property!

def rotm_to_quat(m):
    '''
    Function to convert a rotation matrix to a quaternion

    Input Args: 
    
    Returns:
    '''
    t = np.trace(m)
    q = np.asarray([0.0, 0.0, 0.0, 0.0], dtype=np.float64)

    if t > 0:
        t = np.sqrt(t + 1)
        q[0] = 0.5 * t
        t = 0.5 / t
        q[1] = (m[2,1] - m[1,2]) * t
        q[2] = (m[0,2] - m[2,0]) * t
        q[3] = (m[1,0] - m[0,1]) * t
    else:
        i = 0
        if m[1,1] > m[0,0]:
            i = 1
        if m[2,2] > m[i,i]:
            i = 2
        j = (i+1)%3
        k = (j+1)%3
        t = np.sqrt(m[i,i] - m[j,j] - m[k,k] + 1)
        q[i+1] = 0.5 * t
        t = 0.5 / t
        q[0] = (m[k,j] - m[j,k]) * t
        q[j+1] = (m[j,i] + m[i,j]) * t
        q[k+1] = (m[k,i] + m[i,k]) * t

    return q

def quat_to_rotm(quat):
    '''
    Function to convert a quaternion to a rotation matrix

    Input Args: 
    
    Returns:
    '''
    r = R.from_quat(quat, scalar_first=True)
    return r.as_matrix()

def quat_to_tranform(unit_dual_quat):
    '''
    Function to convert a unit dual quaternion into a 4x4 transformation matrix (element of SE(3))

    Input Args: 
    
    Returns:
    '''
    print(f"unit_dual_quat:\n {unit_dual_quat}")
    quat_r = unit_dual_quat[:, 0:4]
    print(f"quat_r:\n {quat_r}")
    quat_d = unit_dual_quat[:, 4:9]
    print(f"quat_d:\n {quat_d}")
    rotm = quat_to_rotm(quat_r)
    p_quat = 2*quat_prod(quat_d, conjugate_quat(quat_r))
    # print(f"p_quat:\n {p_quat}")
    p = p_quat[1:]
    print(f"p: {p}")
    return [rotm, p]


def conjugate_quat(quat):
    '''
    Function to compute the conjugate of a quaternion.

    Input Args: 
    
    Returns:
    '''
    quat = np.reshape(quat, [1,4])
    quat = np.reshape(quat, [1,4])
    q_0 = quat[:, 0]
    q_r = np.multiply(quat[:, 1:4], -1)
    quat_star = np.reshape(np.append(q_0, q_r+0.0), [1,4])
    return quat_star


def conjugate_dual_quat(dual_quat):
    '''
    Function to compute conjugate of a unit dual quaternion

    Input Args: 
    
    Returns:
    '''
    if dual_quat.shape[1] == 8:
        dual_quat_star = np.asarray([dual_quat[:, 0], -dual_quat[:, 1], -dual_quat[:, 2], -dual_quat[:, 3],
                                dual_quat[:, 4], -dual_quat[:, 5], -dual_quat[:, 6], -dual_quat[:, 7]]) + 0
        return np.reshape(dual_quat_star, [1, 8])
    else:
        print("Incorrect input dimensions!")


def quat_prod(quat_1, quat_2):
    '''
    Function to compute product of two unit quaternions.
    
    Input Args: 
    
    Returns:
    '''
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
    '''
    Function to compute dual quaternion product

    Input Args: 

    Returns:
    
    '''
    p = quat_1[:, 0:4]
    q = quat_1[:, 4:9]
    u = quat_2[:, 0:4]
    v = quat_2[:, 4:9]
    prod = np.append(quat_prod(p, u), (quat_prod(q, u) + quat_prod(p, v)))
    return prod + 0


def get_screw_params(unit_dual_quat):
    '''
    Function to compute screw parameters given a unit dual quaternion corresponding to a relative transformation.
    
    Input Args: 
    Returns:
    
    '''
    # Extracting the real part of the unit dual quaternion:
    quat_r = unit_dual_quat[0:4]
    scalar_quat_r = quat_r[0]
    vector_quat_r = quat_r[1:4]
    if la.norm(vector_quat_r) <= 1e-12:
        u = np.reshape(np.asarray([0, 0, 1]), [1,3])
        theta = 0
    else:
        u = np.divide(vector_quat_r, la.norm(vector_quat_r))
        theta = 2*np.arctan2(la.norm(vector_quat_r), scalar_quat_r)
    # Keeping theta between 0 to pi
    if theta > math.pi:
        theta = 2*math.pi - theta
        u = -u

    # Extracting the dual part:
    quat_d = np.reshape(unit_dual_quat[4:9], [1,4])

    # Computing the translation vector: 
    p_quat = 2*quat_prod(quat_d, conjugate_quat(quat_r))
    p_vect = p_quat[1:4]
    d = np.dot(p_vect, u)
    m = 1/2*(np.cross(p_vect, u) + (p_vect - d*u)*(1/(np.tan(theta/2))))

    # Computing the point on the unit vector. The unit vector along with the point form the screw axis.
    point = np.cross(u, m)
    
    return [theta+0.0, point+0.0, u+0.0, p_vect+0.0, m+0.0, d+0.0]


def sclerp(R_init, p_init, R_final, p_final):
    '''
    Function to perform screw linear interpolation given unit dual quaternion representation of two poses in SE(3)

    Input Args: 
    
    Returns:
    
    '''
    # Convert the rotation matrix into a unit quaternion:
    r_init = R.from_matrix(R_init)
    R_init_quat = np.reshape(r_init.as_quat(scalar_first=True), [1,4])
    # Convert the position vector into a quaternion:
    p_init_quat = np.reshape(np.append([0], p_init), [1,4])
    # Unit dual quaternion of the initial pose:
    g_init_unit_quat = np.reshape(np.append(R_init_quat, 1/2*quat_prod(p_init_quat, R_init_quat)), [1,8])

    # Convert the rotation matrix into a unit quaternion:
    r_final = R.from_matrix(R_final)
    R_final_quat = np.reshape(r_final.as_quat(scalar_first=True), [1,4])
    # Convert the position vector into a quaternion:
    p_final_quat = np.reshape(np.append([0], p_final), [1,4])
    # Unit dual quaternion of the initial pose:
    g_final_unit_quat = np.reshape(np.append(R_final_quat, 1/2*quat_prod(p_final_quat, R_final_quat)), [1,8])

    # Compute the unit dual quaternion corresponding to the relative transformation:
    D = dual_quat_prod(conjugate_dual_quat(g_init_unit_quat), g_final_unit_quat)

    print(f'Computing the screw parameters!')
    # Computing the screw parameters:
    screw_params =  get_screw_params(D)

    print(f'Performing screw interpolation')
    # tau is the interpolation parameter:
    tau = np.arange(0, 1.1, 0.1)

    # Computing the point on the unit vector. The unit vector along with the point form the screw axis.
    theta = screw_params[0]
    point = screw_params[1]
    unit_vector = screw_params[2]
    m = screw_params[4]
    d = screw_params[5]

    # Initializing empty multidimensional arrays to save the computed intermediate interpolated poses:
    R_array, p_array = np.zeros([3,3,len(tau)]), np.zeros([3,1,len(tau)])
    C_dual_quat_array, g_array = np.zeros([8, len(tau)]), np.zeros([4,4,len(tau)])

    for i in range(len(tau)):
        # Computing the real and the dual parts of the unit dual quaternion corresponding to the intermediate 
        # configurations computed using the interpolation scheme.
        # Equations (39) and (44) from Yan-Bin Jia's notes have been used for this purpose
        D_r = np.reshape(np.append(np.cos(tau[i]*theta/2), np.sin(tau[i]*theta/2)*unit_vector), [1, 4])
        D_r[np.isnan(D_r)] = 0
        D_d = np.reshape(np.append(-tau[i]*d/2*np.sin(tau[i]*theta/2), tau[i]*d/2*np.cos(tau[i]*theta/2)*unit_vector + np.sin(tau[i]*theta/2)*m), [1, 4])
        D_d[np.isnan(D_d)] = 0
        C_dual_quat_array[:, i] = dual_quat_prod(g_init_unit_quat, np.reshape(np.append(D_r, D_d), [1, 8]))
        print(f"Dual quaternion interpolation:\n {C_dual_quat_array[:, i]}")

        # Computing the rotation matrix and position vector for a particular configuration from its corresponding 
        # unit dual quaternion representation:
        g = quat_to_tranform(np.reshape(C_dual_quat_array[:, i], [1, 8]))
        R_array[:, :, i], p_array[:, :, i] = np.reshape(g[0], [3,3]), np.reshape(g[1], [3,1])
        # print(f"Rotation matrix: {R_array[:, :, i]}")
        # print(f"position vector: {p_array[:, :, i]}")

    return [R_array, p_array, C_dual_quat_array, screw_params]







