import numpy as np
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