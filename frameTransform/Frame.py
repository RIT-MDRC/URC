"""
DO NOT USE
this is just a reference now
instead use DHTable
"""

from math import *
import numpy as np

class Frame:

    def __init__(self, rot_mat, pos_vec):
        self.create_frame(rot_mat, pos_vec)
        pass

    def create_frame(self, rot_mat, pos_vec):
        # there is definetly a better way to do all of this
        # I don't know enough about numpy though

        self.data = np.array([[0., 0., 0., 0.], [0., 0., 0., 0.], [0., 0., 0., 0.], [0., 0., 0., 0.]])

        for i in range(0, 3):
            for j in range(0, 3):
                self.data[i][j] = rot_mat[i][j]

        for i in range(0, 3):
            self.data[i][3] = pos_vec[i]

        array = np.array([0., 0., 0., 1.])
        np.append(self.data, array)
        for i in range(0, 4):
            self.data[3][i] = array[i]


    def __str__(self):
        return self.data.__str__()


def rot_mat(x: float, y: float, z: float):
    """
    expects angles to be radians
    """
    return get_mat_rotate_x(x) @ get_mat_rotate_y(y) @ get_mat_rotate_z(z)

def get_mat_rotate_x(angle: float):
    """
    expects angle in radians
    """
    return np.array([
        [1, 0, 0], 
        [0, cos(angle), -sin(angle)], 
        [0, sin(angle), cos(angle)]
    ])

def get_mat_rotate_y(angle: float):
    """
    expects angle in radians
    """
    return np.array([
        [cos(angle), 0, sin(angle)],
        [0, 1, 0],
        [-sin(angle), 0, cos(angle)]
    ])

def get_mat_rotate_z(angle: float):
    """
    expects angle in radians
    """
    return np.array([
        [cos(angle), -sin(angle), 0],
        [sin(angle), cos(angle), 0],
        [0, 0, 1]
    ])