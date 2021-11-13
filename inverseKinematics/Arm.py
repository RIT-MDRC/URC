import numpy as np

# @param k: the unit axis (kx,ky,kz) 
# @param q: angle in radians to rotate by
def axis_angle_rot_matrix(axis, angle):

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

if __name__ == '__main__':
  main()