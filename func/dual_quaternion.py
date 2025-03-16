# Class for unit dual quaternions.
# By Aditya Patankar

import numpy as np

class unit_dual_quaternion:

    def __init__(self):
        # A 1x4 numpy array
        self.dual_quat = None
        # Conjugate of the unit dual quaternion also a 1x8 numpy array
        self.conj_dual_quat = None

    def conjugate_dual_quat(self):
        '''
        Function to compute conjugate of a unit dual quaternion
        '''
        if self.dual_quat.shape[1] == 8:
            dual_quat_star = np.asarray([self.dual_quat[:, 0], -self.dual_quat[:, 1], -self.dual_quat[:, 2], -self.dual_quat[:, 3],
                                    self.dual_quat[:, 4], -self.dual_quat[:, 5], -self.dual_quat[:, 6], -self.dual_quat[:, 7]]) + 0
            self.conj_dual_quat =  np.reshape(dual_quat_star, [1, 8])
        else:
            print("Incorrect input dimensions!")
