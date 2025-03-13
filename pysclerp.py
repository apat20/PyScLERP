import numpy as np
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
Function to compute conjugate of a unit dual quaternion
'''
def conjugate_dual_quat(quat):
    if quat.shape[1] == 8:
        quat_star = np.asarray([quat[:, 0], -quat[:, 1], -quat[:, 2], -quat[:, 3],
                                quat[:, 4], -quat[:, 5], -quat[:, 6], -quat[:, 7]])
        return np.reshape(quat_star, [1, 8])
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
    return prod

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
    return prod

'''
Function to compute screw parameters given a unit dual quaternion
'''
def get_screw_params():
    return None

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
    R_init_quat = -1*np.reshape(r_init.as_quat(scalar_first=True), [1,4])
    # Convert the position vector into a quaternion:
    p_init_quat = np.reshape(np.append([0], p_init), [1,4])
    # Unit dual quaternion of the initial pose:
    g_init_unit_quat = np.reshape(np.append(R_init_quat, 1/2*quat_prod(p_init_quat, R_init_quat)), [1,8])

    breakpoint()

    # Final configuration:
    R_final = np.asarray([[1, 0, 0],
                          [0, 0, 1],
                          [0, -1, 0]])
    p_final = np.asarray([0, 20, 10])

    # Convert the rotation matrix into a unit quaternion:
    r_final = R.from_matrix(R_final)
    R_final_quat = -1*np.reshape(r_final.as_quat(scalar_first=True), [1,4])
    # Convert the position vector into a quaternion:
    p_final_quat = np.reshape(np.append([0], p_final), [1,4])
    # Unit dual quaternion of the initial pose:
    g_final_unit_quat = np.reshape(np.append(R_final_quat, 1/2*quat_prod(p_final_quat, R_final_quat)), [1,8])

    
    breakpoint()

    D = dual_quat_prod(conjugate_dual_quat(g_init_unit_quat), g_final_unit_quat)

    breakpoint()



