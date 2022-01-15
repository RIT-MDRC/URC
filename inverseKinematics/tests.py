import numpy as np

from Arm import *

def test1(verbose):
    # axes of rotation for each joint
    # k = kx, ky, kz
    k = np.array([[0, 0, 1],[0, 0, 1]])

    # A 2D array that lists the translations from the previous joint to the current joint
    # The first translation is from the base frame to joint 1 (which is equal to the base frame)
    # The second translation is from joint 1 to joint 2
    # t = tx, ty, tz
    # using approximate measurements for now
    # get the actual measurements from the cad model
    a1 = 4.7
    a2 = 5.9
    a3 = 5.4
    a4 = 6.0
    t = np.array([[0, 0, 0], [a2, 0, a1]])

    # Position of end effector in the last joint's frame
    endeffector_position = [a4, 0, a3]

    arm = Arm(k,t)

    # Starting joint angles in radians
    starting_joints = np.array([0, 0])

    # desired end position for the end effector with respect to the base frame of the robotic arm
    # endeffector_goal_position = np.array([4.0,10.0,a1 + a4])
    endeffector_goal_position = np.array([4.0, 10.0, a1 + a4])

    # Display the starting position of each joint in the global frame
    if verbose:
        for i in np.arange(0, arm.n):
            print(f'joint {i} position = {arm.position(starting_joints, i)}')

        print(f'end_effector = {arm.position(starting_joints, -1, endeffector_position)}')
        print(f'goal = {endeffector_goal_position}')

    # return the final joint positions
    # hopefully the goal position is reasonable and is reached
    final_q = arm.pseudo_inverse(starting_joints,
                                 endeffector_position,
                                 endeffector_goal_position,
                                 500)

    # Final Joint Angles in degrees
    print('\nFinal Joint Angles in Degrees')
    print(f'Joint 1: {np.degrees(final_q[0])} , Joint 2: {np.degrees(final_q[1])}')
    print(f'Expected: Joint 1: ~43 , Joint 2: ~50')
