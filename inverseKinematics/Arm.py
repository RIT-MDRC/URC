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
@param axis: A 3 element array containing the unit axis to rotate around (kx,ky,kz)
@param t_mat: The translation from the current frame (e.g. Frame A) to the next frame (e.g. Frame B)
@param q: The rotation angle (i.e. joint angle)
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

if __name__ == '__main__':
  main()