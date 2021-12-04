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

def main():
    pass

if __name__ == '__main__':
  main()