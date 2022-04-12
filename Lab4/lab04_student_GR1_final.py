import numpy as np

#######################
######## Lab 4 ########
#######################

def traj_gen_config(q1, q2, t, Tf):
    """
    path plan configuration space

    Args:
        q1: Start configuration (angles) [degrees] --> np.array((6,))
        q2: Goal configuration (angles) [degrees] --> np.array((6,))
        t: Time instance --> float
        Tf: Total movement time --> float

    Returns: angles positions, angles velocities, angles accelerations
                --> q, dq, ddq --> 3 object of np.array((6,))
    
    B.C: q(0) = q1; q_dot(0) = 0; q(Tf) = q2; q_dot(Tf) = 0; 
    """
    a0 = q1
    a1 = 0.
    a2 = (3 / (Tf ** 2)) * (q2 - q1)
    a3 = (2 / (Tf ** 3)) * (q1 - q2)
    q = a0 + a1 * t + a2 * (t ** 2) + a3 * (t ** 3)
    dq = a1 + 2 * a2 * t + 3 * a3 * (t ** 2)
    ddq = 2 * a2 + 6 * a3 * t
    
    '''
    linear: a0 = q1 ; a1 = (q2 - q1) / Tf ; q = a0 + a1 * t ; dq = a1 ; ddq = 0;
    '''
    return q, dq, ddq

def traj_gen_task(x_s, x_g, t, Tf):
    """
    path plan in Task space

    Args:
        x_s: Start end-effector position and orientation UNITS:[m, degrees] --> np.array((6,))
        x_g: Goal end-effector position and orientation UNITS:[m, degrees] --> np.array((6,))
        t: Time instance --> float
        Tf: Total movement time --> float

    Returns: End-effector position, velocity, acceleration
                --> x, dx, ddx --> 3 object of np.array((6,))

    """
    a0 = x_s
    a1 = 0.
    a2 = (3 / (Tf ** 2)) * (x_g - x_s)
    a3 = (2 / (Tf ** 3)) * (x_s - x_g)
    x = a0 + a1 * t + a2 * (t ** 2) + a3 * (t ** 3)
    dx = a1 + 2 * a2 * t + 3 * a3 * (t ** 2)
    ddx = 2 * a2 + 6 * a3 * t

    '''
    linear: a0 = x_s ; a1 = (x_g - x_s) / Tf ; x = a0 + a1 * t ; dx = a1 ; ddx = 0;
    '''

    return x, dx, ddx


def generate_x_goals_list():
    """

    Returns: list of

    """
    x_ = np.array([[0.415, 0.335, 0.615, 90.4, 0.1, 166],
                   [0.445, -0.057, 0.281, 169.759, 0.238, 95.399],
                   [0.434, -0.049, 0.006, 175.89, 0.755, 93.778],
                   [0.467, -0.224, 0.159, 81.521, -0.95, 108.473]])
    return x_


def generate_q_goals_list():
    """

    Returns: list of

    """
    jointPoses = np.array([[22.41, 319.858, 358.647, 329.983, 291.341, 57.045],
                           [0.962, 325.185, 45.99, 270.511, 271.036, 85.574],
                           [2.779, 304.625, 76.142, 269.026, 315.635, 89.646],
                           [308.182, 290.667, 68.802, 20.536, 276.567, 310.149]])

    return jointPoses


'''
rec1:
    x_ = np.array([[0.415, 0.335, 0.615, 90.4, 0.1, 166],
                   [0.445, -0.057, 0.281, 169.759, 0.238, 95.399],
                   [0.434, -0.049, 0.006, 175.89, 0.755, 93.778],
                   [0.467, -0.224, 0.159, 81.521, -0.95, 108.473]])
    return x_

    jointPoses = np.array([[22.41, 319.858, 358.647, 329.983, 291.341, 57.045],
                           [0.962, 325.185, 45.99, 270.511, 271.036, 85.574],
                           [2.779, 304.625, 76.142, 269.026, 315.635, 89.646],
                           [308.182, 290.667, 68.802, 20.536, 276.567, 310.149]])
  
rec2:
    x_ = np.array([[0.44, 0.187, 0.419, 96, 1, 150],
                   [0.463, -0.131, 0.298, 99.211, -175.981, 99.796],
                   [0.634, -0.14, 0.297, 87.317, -177.205, 92.726],
                   [0.305, 0.403, 0.53, 105.716, -159.563, 171.653]])
    return x_


    jointPoses = np.array([[0.1, 343, 75, 354, 300, 0.1],
                           [334.71, 325.18, 122.117, 234.212, 80.389, 279.347],
                           [347.508, 311.931, 77.55, 243.598, 35.784, 288.416],
                           [37.575, 340.48, 90.23, 211.41, 59.793, 304.981]])
'''