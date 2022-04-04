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

    Returns: Desired end-effector goals along the planned path --> np.array((4, 6))

    Notes:  1. Position units [m]
            2. Orientation units [degrees]

    """
    pass


def generate_q_goals_list():
    """

    Returns: Desired configuration (angle) goals along the planned path --> --> np.array((4, 6))

    Notes: Orientation units [degrees]

    """

    pass
