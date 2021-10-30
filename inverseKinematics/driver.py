from math import *
import numpy as np

from DHTable import *

"""
creates the table representing the actual arm
the measurements are not correct but they can easily be changed

    theta_i     alpha_i     r_i     d_i
1   theta_1     90          0       a_1
2   theta_2     0           a_2     0

calculates and prints the transformation matrices
"""

def main():

    table = DHTable()
    table.append_element(100, radians(90), 0, radians(0))
    table.append_element(432, radians(0), 0, radians(90))
    print(table)
    print()
    print(table.get_transform_mat_at(0))
    print()
    print(table.get_transform_mat_at(1))


if __name__ == '__main__':
    main()
