from math import *
import numpy as np

"""
z_distance  distance from Zi to Zi+1, measured along Xi
z_angle     angle from Zi to Zi+1, measured about Xi

x_distance  distance from Xi to Xi+1, measured along Zi
x_angle     angle from Xi to Xi+1, measured about Zi

counter clockwise is positive
clockwise is negative
"""

class DHTable:

    def __init__(self):
        self.table = []

    def append_element(self, z_angle, x_angle, x_distance, z_distance):
        self.table.append(Element(len(self.table) + 1, z_distance, z_angle, x_distance, x_angle))

    def get_transform_mat_from(self, a1, a2):
        base = self.get_transform_mat_at(a1)
        for i in range(a1+1, a2):
            base = base @ self.get_transform_mat_at(i)
        return base

    def get_transform_mat_at(self, a1):
        a1 = self.table[a1]
        return np.array([
            [cos(a1.x_angle),                    -sin(a1.x_angle),                    0,                   a1.z_distance], 
            [sin(a1.x_angle) * cos(a1.z_angle),  cos(a1.x_angle) * cos(a1.z_angle),   -sin(a1.z_angle),    -sin(a1.z_distance) * a1.x_distance], 
            [sin(a1.x_angle) * sin(a1.z_angle),  cos(a1.x_angle) * sin(a1.z_angle),   cos(a1.z_angle),     cos(a1.z_distance) * a1.x_distance], 
            [0,                                  0,                                   0,                   1], 
        ])

    def __str__(self) -> str:
        str = '\tz_d\tz_a\tx_d\tx_a\t\n'
        for el in self.table:
            str += el.__str__()
        return str


class Element:

    def __init__(self, axis, z_angle, x_angle, x_distance, z_distance):
        self.axis = axis
        self.z_distance = z_distance
        self.z_angle = z_angle
        self.x_distance = x_distance
        self.x_angle = x_angle

    def __str__(self) -> str:
        return '' + str(self.axis) + '|\t' + str(self.z_angle) + '\t' + str(self.x_angle) + '\t' + str(self.x_distance) +'\t' + str(self.z_distance) + '\t\n'
