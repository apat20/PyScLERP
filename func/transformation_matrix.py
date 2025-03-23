# Class for transformation matrices, poses or rigid body configurations which are elements of SE(3)
# By Aditya Patankar

import numpy as np

class transformation_matrix:

    def __init__(self, matrix):
        # 4x4 transformation matrix, which is an element of SE(3)
        self.trans_mat = matrix
        # 3x3 rotation matrix, which is an element of SO(3)
        self.R = self.trans_mat[0:3, 0:3]
         # 3x1 position vector, element of R^3
        self.p = np.reshape(self.trans_mat[0:3, 3], [3,1])

    def inverse_trans_mat(self):
        """
        Function to compute the inverse of a transformation matrix.
        """

        if self.matrix.shape == [4,4]:
            print('compute inverse')
        else:
            print('Incorrect dimensions')
    
    def compute_adjoint(self):
        """
        Function to compute the adjoint of a transformation matrix.
        """
        if self.matrix.shape == [4,4]:
            print('compute inverse')

    
    def plot_reference_frame(self, scale_value, length_value, ax):
        """
        Function to plot the reference frame corresponding to a rigid body pose or configuration.

        Args: 
        
        Returns:
        """
        if self.R.shape == [3,3] and self.p.shape == [3,1]:
            ax.quiver(self.p[0, :], self.p[1, :], self.p[2, :], scale_value*self.R[0, 0], scale_value*self.R[1, 0], scale_value*self.R[2, 0], color = "r", arrow_length_ratio = length_value)
            ax.quiver(self.p[0, :], self.p[1, :], self.p[2, :], scale_value*self.R[0, 1], scale_value*self.R[1, 1], scale_value*self.R[2, 1], color = "g", arrow_length_ratio = length_value)
            ax.quiver(self.p[0, :], self.p[1, :], self.p[2, :], scale_value*self.R[0, 2], scale_value*self.R[1, 2], scale_value*self.R[2, 2], color = "b", arrow_length_ratio = length_value)
            return ax
        else:
            print('Incorrect input dimensions')
