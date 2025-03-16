# Class for quaternions.
# By Aditya Patankar

import numpy as np
from scipy.spatial.transform import Rotation as R

class unit_quaternion:

    def __init__(self):
        # A 1x4 numpy array (w, x, y, z)
        self.quat = None
        # Conjugate of a quaternion also a 1x4 numpy array (w, x, y, z)
        self.conj_quat = None
        # Scalar part of the quaternion. Scalar first convention is followed.
        self.q_0 = self.quat[:, 0]
        # Vector part of the quaternion:
        self.q_r = np.multiply(self.quat[:, 1:4], -1)
        self.rotm = None

    def quat_to_rotm(self):
        '''
        Function to convert a quaternion to a rotation matrix
        '''
        if self.quat.shape == [1,4]:
            r = R.from_quat(self.quat, scalar_first=True)
            self.rotm = r.as_matrix()
    
    def conjugate_quat(self):
        '''
        Function to compute the conjugate of a quaternion.
        '''
        if self.quat.shape == [1,4]:
            self.conj_quat = np.reshape(np.append(self.q_0, self.q_r+0.0), [1,4])
