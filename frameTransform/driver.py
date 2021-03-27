from math import *
import numpy as np

from DHTable import *

"""
creates the table representing the actual arm
the measurements are not correct but they can easily be changed

ok ngl I have no idea wtf I'm doing here

z_d     z_a     x_d     x_a
----------------------------
100     0       0       0
432     90      0       90
625     0       0       0
80      0       0       0
30      0       0       0
100     0       0       0


calculates and prints the transformation matrices
"""
def main():

    table = DHTable()
    table.append_element(100, radians(0), 0, radians(0))
    table.append_element(432, radians(90), 0, radians(90))
    table.append_element(625, radians(0), 0, radians(0))
    table.append_element(80, radians(0), 0, radians(0))
    table.append_element(30, radians(0), 0, radians(0))
    table.append_element(100, radians(0), 0, radians(0))
    print(table)
    print()
    print(table.get_transform_mat_at(0))
    print()
    print(table.get_transform_mat_at(1))
    print()
    print(table.get_transform_mat_at(2))
    print()
    print(table.get_transform_mat_at(3))
    print()
    print(table.get_transform_mat_at(4))
    print()
    print(table.get_transform_mat_at(5))


if __name__ == '__main__':
    main()
