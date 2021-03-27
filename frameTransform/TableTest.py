"""
DO NOT USE
this us just a test to make sure the transformation
matrices are being calculated correctly
"""

from math import *
import numpy as np

from DHTable import *

"""
creates the table
z_d     z_a     x_d     x_a
----------------------------
0       0       1       0       # prismatic
10      0       0       30      # revolute
5       0       0       60      # revolute

then calculate the transformation matrices from (0,1), (1,2), (2,0)
"""
def main():

    table = DHTable()
    table.append_element(0., radians(0.), 1., radians(0.))
    table.append_element(10., radians(0.), 0., radians(30.))
    table.append_element(15., radians(0.), 0., radians(60.))
    print(table)
    print()
    print(table.get_transform_mat_at(0))
    print()
    print(table.get_transform_mat_at(1))
    print()
    print(table.get_transform_mat_at(2))


if __name__ == '__main__':
    main()