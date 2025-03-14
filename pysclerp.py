import numpy as np
from numpy import linalg as la
import math 
from scipy.spatial.transform import Rotation as R

# NOTE: Quaternions -q and q represent the same rotation? Need to check this property!

'''
Function to compute the skew symmetric form of a matrix
'''
def skew_symmetric():
    return None

'''
Function to compute the inverse of a transformation matrix (element of SE(3))
'''
def inverse_trans_mat():
    return None

'''Function to convert a rotation matrix to a quaternion'''
def rotm_to_quat(m):
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

'''
Function to compute the conjugate of a quaternion:
'''
def conjugate_quat(quat):
    q_0 = np.asarray(quat[0])
    q_r = quat[1:4]
    quat_star = np.reshape(np.append(q_0, -q_r+0.0), [1,4])
    return quat_star

'''
Function to compute conjugate of a unit dual quaternion
'''
def conjugate_dual_quat(dual_quat):
    if dual_quat.shape[1] == 8:
        dual_quat_star = np.asarray([dual_quat[:, 0], -dual_quat[:, 1], -dual_quat[:, 2], -dual_quat[:, 3],
                                dual_quat[:, 4], -dual_quat[:, 5], -dual_quat[:, 6], -dual_quat[:, 7]]) + 0
        return np.reshape(dual_quat_star, [1, 8])
    else:
        print("Incorrect input dimensions!")
    

'''
Function to convert a unit dual quaternion into a 4x4 transformation matrix (element of SE(3))
'''
def quat_2_tranform():
    return None

'''
Function to compute product of two unit quaternions:
'''
def quat_prod(quat_1, quat_2):
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

'''
Function to compute dual quaternion product
'''
def dual_quat_prod(quat_1, quat_2):
    p = quat_1[:, 0:4]
    print(f"shape of p: {p.shape}")
    q = quat_1[:, 4:9]
    print(f"shape of q: {q.shape}")
    u = quat_2[:, 0:4]
    print(f"shape of u: {u.shape}")
    v = quat_2[:, 4:9]
    print(f"shape of v: {v.shape}")
    prod = np.append(quat_prod(p, u), (quat_prod(q, u) + quat_prod(p, v)))
    return prod + 0

'''
Function to compute screw parameters given a unit dual quaternion
'''
def get_screw_params(unit_dual_quat):
    # Extracting the real part of the dual quaternion:
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
    p = p_quat[1:4]
    d = np.dot(p, u)
    m = 1/2*(np.cross(p, u) + (p - d*u)*(1/(np.tan(theta/2))))
    return [theta+0.0, u+0.0, p+0.0, m+0.0, d+0.0]

'''
Function to perform screw linear interpolation given unit dual quaternion representation of two poses in SE(3)
'''
def sclerp():
    return None


if __name__ == '__main__':

    print(f'Inside the main function!')

    # Initial configuration:
    R_init = np.eye(3)
    p_init = np.zeros([1,3])

    # Convert the rotation matrix into a unit quaternion:
    r_init = R.from_matrix(R_init)
    R_init_quat = np.reshape(r_init.as_quat(scalar_first=True), [1,4])
    # Convert the position vector into a quaternion:
    p_init_quat = np.reshape(np.append([0], p_init), [1,4])
    # Unit dual quaternion of the initial pose:
    g_init_unit_quat = np.reshape(np.append(R_init_quat, 1/2*quat_prod(p_init_quat, R_init_quat)), [1,8])

    # Final configuration:
    R_final = np.asarray([[1, 0, 0],
                          [0, 0, 1],
                          [0, -1, 0]])
    p_final = np.asarray([0, 20, 10])

    # Convert the rotation matrix into a unit quaternion:
    r_final = R.from_matrix(R_final)
    R_final_quat = np.reshape(r_final.as_quat(scalar_first=True), [1,4])
    # Convert the position vector into a quaternion:
    p_final_quat = np.reshape(np.append([0], p_final), [1,4])
    # Unit dual quaternion of the initial pose:
    g_final_unit_quat = np.reshape(np.append(R_final_quat, 1/2*quat_prod(p_final_quat, R_final_quat)), [1,8])

    D = dual_quat_prod(conjugate_dual_quat(g_init_unit_quat), g_final_unit_quat)

    breakpoint()

    print(f'Computing the screw parameters!')

    # Computing the screw parameters:
    screw_params =  get_screw_params(D)


    breakpoint()



