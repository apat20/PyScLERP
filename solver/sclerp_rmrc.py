import numpy as np
from numpy import linalg as la
from scipy.spatial.transform import Rotation as R

import func.quaternion_lib as ql
import func.kinematics_lib as kl

class ScLERPController:
    def __init__(self, goal_pose, dt=1.0/60.0, beta=0.1):
        """
        Initialize the SclERP-RMRC Controller with robot parameters and the goal pose

        Args:

        
        """
        # Task parameters:
        self.goal_pose = goal_pose
        self.dt = dt
        self.beta = beta
        self.counter = 0
        self.goal_not_reached = True

        # Preprocess the rotation matrix and convert to Unit dual quaternion:
        r_goal = R.from_matrix(self.goal_pose[:3, :3])
        q_goal_scipy = r_goal.as_quat()
        self.R_goal_quat = np.reshape(np.array([q_goal_scipy[3], q_goal_scipy[0], q_goal_scipy[1], q_goal_scipy[2]]), [1, 4])
        # Convert the position vector into a quaternion:
        self.p_goal_quat = np.reshape(np.append([0], self.goal_pose[:3, 3]), [1,4])
        # Unit dual quaternion of the initial pose:
        self.g_goal_unit_dq = np.reshape(np.append(self.R_goal_quat, 1/2*ql.quat_prod(self.p_goal_quat, self.R_goal_quat)), [1,8])

        # Setting the robot parameters: 
        self._initialize_baxter()

    
    def _initialize_baxter(self):
        """
        Define joint axes w, joint origins q and gst_0. 
        Currently defined for Baxter

        """
        # Joint origins (With respect to the Robot's base)
        q1 = np.asarray([0.056,0.000,0.011])
        q2 = np.asarray([0.125,0.000,0.281])
        q3 = np.asarray([0.227,0.000,0.281])
        q4 = np.asarray([0.489,0.000,0.213])
        q5 = np.asarray([0.593,-0.001,0.213])
        q6 = np.asarray([0.863,-0.001,0.195])
        q7 = np.asarray([0.979,-0.002,0.195])
        qr = np.asarray([q1, q2, q3, q4, q5, q6, q7])

        # Joint axes (With respect to the Robot's base)
        w1 = np.asarray([0,0,1])
        w2 = np.asarray([0,1,0]) 
        w3 = np.asarray([1,0,0])

        w4, w5, w6, w7 = w2, w3, w2, w3
        wr = np.asarray([w1, w2, w3, w4, w5, w6, w7])

        # Base transforms
        g_st0_right = np.asarray([[0,  0, 1, 1.213],
                                [0,  1, 0, -0.002],
                                [-1, 0, 0, 0.190],
                                [0,  0, 0, 1.0000]])
        
        g_base_right = np.asarray([[0.7071, 0.7071,  0, 0.025],
                                [-0.7071, 0.7071, 0, -0.220],
                                [0,       0,      1,  0.109],
                                [0,       0,      0,  1.0]])

        # Transform to base frame
        self.wr_r = g_base_right[:3, :3] @ wr.T
        self.qr_r = np.add(g_base_right[:3, :3] @ qr.T, np.reshape(g_base_right[:3, 3], [3, 1]))
        self.gst0_r = g_base_right @ g_st0_right
        self.num_joints = 7

    def reset(self):
        """
        Resets the solver state for a new episode
        """
        self.counter = 0
        self.goal_not_reached = True

    def step(self, current_joint_position):
        """
        The Main Control Loop Function which tries to reach the goal from the current position
        
        Args:

        Output:

        """
        theta_curr = current_joint_position
        g_curr = kl.forward_kinematics(self.gst0_r, theta_curr, self.wr_r, self.qr_r)
        _, J_spatial = kl.compute_spatial_jacobian(theta_curr, self.wr_r, self.qr_r, self.gst0_r)

        # ScLERP-based Planning
        # Plan from the current pose to the goal, then take the FIRST step (tau = 0.1)
        R_curr, p_curr = g_curr[:3, :3], g_curr[:3, 3]

        # NOTE: Calling ql.sclerp returns the whole trajectory. 
        # We need only the frame at index 1. 
        _, _, _, G_array, _ = ql.sclerp(R_curr, p_curr,
                                        self.goal_pose[:3, :3],
                                        self.goal_pose[:3, 3],
                                        tau = 0.1) # tau defines the step size.
        
        target_pose_next = G_array[:, :, 1]

        # Compute the next joint position:
        theta_next = self._compute_next_theta(J_spatial, g_curr, target_pose_next, theta_curr)
        g_next = kl.forward_kinematics(self.gst0_r, theta_next, self.wr_r, self.qr_r)

        # Check for for convergence:
        rotation_err, position_err = self._compute_error(g_next)

        if rotation_err < 0.07 and position_err < 0.002:
            self.goal_not_reached = False
            print(f"Goal reached at step {self.counter}!")
        
        self.counter += 1
        return theta_next, not self.goal_not_reached

    # HELPER FUNCTIONS: 

    def _compute_error(q1, q2):
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
    
    def _compute_next_theta(self, spatial_jacobian, g_curr, g_next, theta_curr):
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
 
        # Final pose:
        R_next, p_next = g_next[0:3, 0:3], g_next[0:3, 3]
        # Convert the rotation matrix into a unit quaternion:
        r_next = R.from_matrix(R_next)
        q_next_scipy = r_next.as_quat()
        R_next_quat = np.reshape(np.array([q_next_scipy[3], q_next_scipy[0], q_next_scipy[1], q_next_scipy[2]]), [4])

        # --- CRITICAL FIX 3: Ensure Quaternion Continuity ---
        if np.dot(R_curr_quat, R_next_quat) < 0:
            R_next_quat = -R_next_quat

        gamma_curr = np.asarray([p_curr[0], p_curr[1], p_curr[2], R_curr_quat[0], R_curr_quat[1], R_curr_quat[2], R_curr_quat[3]])
        gamma_next = np.asarray([p_next[0], p_next[1], p_next[2], R_next_quat[0], R_next_quat[1], R_next_quat[2], R_next_quat[3]])

        p_curr_hat = ql.skew_symmetric(np.reshape(p_curr, [3,1]))

        j1 = self._get_quat_jacobian(R_curr_quat)
        
        j2 = np.zeros([6, 7])
        j2[:3, :3], j2[3:6, :3] = np.eye(3), np.zeros([3,3])
        j2[:3, 3:7], j2[3:7, 3:7] = (2 * p_curr_hat) @ j1, 2 * j1

        # B = spatial_jacobian.T @ la.solve(spatial_jacobian @ spatial_jacobian.T, j2)

        # --- CRITICAL FIX 2: Use PINV instead of solve for stability ---
        # B = J^T * inv(J * J^T) * j2 is equivalent to pinv(J) * j2 for full rank J
        # Using pinv is safer for numerical stability
        B = np.linalg.pinv(spatial_jacobian) @ j2

        delta = self.beta * B @ (gamma_next -  gamma_curr)
        theta_next = theta_curr + self._theta_threshold(delta)

        return theta_next
    
    def _get_quat_jacobian(self, q):
        """
        Computes j1 required for B

        Args:

        Output:
        """
        w, x, y, z = q
        return np.array([[-x, w, -z, y], [-y, z, w, -x], [-z, -y, x, w]])
    
    def _theta_threshold(self, theta_raw):
        """
        Bound the velocity of the joints by SCALING the vector.
        Preserves direction of motion.

        Args:

        Output:

        """
        limit = 0.01
        max_val = np.max(np.abs(theta_raw))
        if max_val > limit:
            theta_raw = theta_raw * (limit / max_val)
        return theta_raw



