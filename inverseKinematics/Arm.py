import numpy as np
from numpy.core.numeric import identity

"""
@param k: the unit axis (kx,ky,kz)
@param q: angle in radians to rotate by
"""
def axis_angle_to_r_mat(axis, angle):

    x = axis[0]
    y = axis[1]
    z = axis[2]
    sin = np.sin(angle)
    cos = np.cos(angle)
    theta = 1 - np.cos(angle)

    return np.array([
        [x * x * theta + np.cos(angle), x * y * theta - z * sin, x * z * theta + y * sin],
        [x * y * theta + z * sin, y * y * theta + cos, y * z * theta - x * sin],
        [x * z * theta - y * sin, y * z * theta + x * sin, z * z * theta + cos]])


"""
@param axis: unit axis (kx,ky,kz)
@param t_mat: translation from the current frame to the next frame
@param q: rotation/joint angle
"""
def homogenous_transform_matrix(axis, t_mat, angle):
    r_mat = axis_angle_to_r_mat(axis, angle)

    t_vec = t_mat

    t_vec = np.array([[t_vec[0]],
                    [t_vec[1]],
                    [t_vec[2]]])

    h_mat = np.concatenate((r_mat, t_vec), axis = 1)
    h_mat = np.concatenate((h_mat, np.array([[0, 0, 0, 1]])), axis = 0)
    return h_mat

"""
da arm
"""
class Arm:
    def __init__(self, axes_of_rotation, translations):
        self.r = np.array(axes_of_rotation)
        self.t = np.array(translations)

        assert axes_of_rotation.shape == translations.shape

        self.n = len(axes_of_rotation)


    """
    Compute the position in the global (base) frame of a point given in a joint frame
    @param p_i: position vector
    @param index: joint index
    
    @return: position vector relative to the global frame
    """
    def position(self, Q, index=-1, p_i = [0,0,0]):

        pos = np.array([
            [p_i[0]],
            [p_i[1]],
            [p_i[2]],
            [1]
        ])

        # end effector
        if (index == -1):
            index = self.N_joints - 1

        start_joint = index
        mat = None

        while (index >= 0):
            if (index == start_joint):
                mat = homogenous_transform_matrix(self.k[index], self.t[index], Q[index]) @ pos
            else:
                mat = homogenous_transform_matrix(self.k[index], self.t[index], Q[index]) @ mat
            index = index - 1

        return np.array([
            mat[0][0],
            mat[1][0],
            mat[2][0]
        ])

    """
    IK solver using the pseudo inverse jacobian method

    @param start_angles: the starting joint angles
    @param end_eff_translation_vector: translation vector from the last joint to the end effector
    @param goal: x, y, z of the goal position
    
    @return: array containing the final joint angles
    """
    def pseudo_inverse(self, start_angles, end_eff_translation_vector, goal, temp):
        angles = start_angles
        
        # these can be tuned as needed
        step_size = 0.05
        max_step = 0.2

        # end: where we want the end effector to end up in the base frame
        # current: where the end effector currently is in the global frame
        # delta: the delta between end and current
        end = np.array([goal[0], goal[1], goal[2]])
        current = self.position(angles, p_i = end_eff_translation_vector)
        delta = end - current

        while np.linalg.norm(delta) > 0.01:
            # print(f'Q[{angles}] , P[{delta}]')

            # reduce by scaling factor
            scaled_delta = delta * step_size / np.linalg.norm(delta)

            jacobian = self.jacobian(angles, end_eff_translation_vector)
            psuedo_inverse_jacobian = np.linalg.pinv(jacobian)
            result = np.matmul(psuedo_inverse_jacobian, scaled_delta)

            # move joints to the new calculated angles
            angles = angles + np.clip(result, -1 * max_step, max_step)

            # calculate the new end effector position
            current = self.position(angles, p_i = end_eff_translation_vector)
            delta = end - current

        return angles;


    """
    Computes the Jacobian matrix

    @param Q: n element array of current join angles (radians)
    @param end_eff_translation_vector: translation vector last joint to end effector relative to the last joint

    @return: 3xN 2D Jacobian matrix
    """
    def jacobian(self, Q, end_eff_translation_vector=[0,0,0]):
        end_effector_global_pos = self.position(Q, -1, end_eff_translation_vector)
        jacobian = []

        joint_offset = end_effector_global_pos - self.position(Q,index=0)

        # Axes
        kx = self.k[0][0]
        ky = self.k[0][1]
        kz = self.k[0][2]
        k = np.array([kx, ky, kz])

        px = joint_offset[0]
        py = joint_offset[1]
        pz = joint_offset[2]
        joint_offset = np.array([px, py, pz])

        this_jacobian = np.cross(k, joint_offset)

        # Convert to a 2D matrix
        j0 = this_jacobian[0]
        j1 = this_jacobian[1]
        j2 = this_jacobian[2]
        this_jacobian = np.array([[j0],
                                [j1],
                                [j2]])
        jacobian = this_jacobian

        for i in range(1, self.n):
            joint_offset = end_effector_global_pos - self.position(Q,index=i)

            # Axes
            kx = self.k[i][0]
            ky = self.k[i][1]
            kz = self.k[i][2]
            k = np.array([kx, ky, kz])

            # FIXME lmao I didn't even notice this
            px = joint_offset[0]
            py = joint_offset[1]
            pz = joint_offset[2]
            joint_offset = np.array([px, py, pz])

            this_jacobian = np.cross(k, joint_offset)

            # Convert to a 2D matrix
            j0 = this_jacobian[0]
            j1 = this_jacobian[1]
            j2 = this_jacobian[2]
            this_jacobian = np.array([[j0],
                                    [j1],
                                    [j2]])
            jacobian = np.concatenate((jacobian, this_jacobian), axis=1) # side by side
        return jacobian;

def main():
    pass

if __name__ == '__main__':
  main()