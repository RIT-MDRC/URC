import numpy as np

from Arm import *

def test1(verbose):
    k = np.array([[0, 0, 1],[0, 0, 1]])

    a1 = 4.7
    a2 = 5.9
    a3 = 5.4
    a4 = 6.0
    t = np.array([[0, 0, 0], [a2, 0, a1]])

    endeffector_position = [a4, 0, a3]

    arm = Arm(k,t)

    starting_joints = np.array([0, 0])

    endeffector_goal_position = np.array([4.0, 10.0, a1 + a4])

    if verbose:
        for i in np.arange(0, arm.n):
            print(f'joint {i} position = {arm.position(starting_joints, i)}')

        print(f'end_effector = {arm.position(starting_joints, -1, endeffector_position)}')
        print(f'goal = {endeffector_goal_position}')

    final_q = arm.pseudo_inverse(starting_joints,
                                 endeffector_position,
                                 endeffector_goal_position,
                                 500)

    print('\nFinal Joint Angles in Degrees')
    print(f'Joint 1: {np.degrees(final_q[0])} , Joint 2: {np.degrees(final_q[1])}')
    print(f'Expected: Joint 1: ~43 , Joint 2: ~50')