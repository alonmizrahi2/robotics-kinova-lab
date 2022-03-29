import numpy as np
from sympy import *
from scipy.spatial.transform import Rotation

alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha1:7')
a1, a2, a3, a4, a5, a6 = symbols('a1:7')
d1, d2, d3, d4, d5, d6 = symbols('d1:7')
q1, q2, q3, q4, q5, q6 = symbols('q1:7')

'''
Hint1: you should use your functions from the previous lab
Hint2: using sympy is easier for debugging, but not mandatory
'''
# from Lab 02
def set_dh_table():
    """

    Returns: dictionary

    Important: all length arguments in [m]
               all angles argument in [radians]

    """
    dh_subs_dict = {alpha1: pi / 2, a1: 0, d1: 0.2433, q1: q1,
                    alpha2: pi, a2: 0.28, d2: 0.03, q2: q2 + pi / 2,
                    alpha3: pi / 2, a3: 0, d3: 0.02, q3: q3 + pi / 2,
                    alpha4: pi / 2, a4: 0, d4: 0.245, q4: q4 + pi / 2,
                    alpha5: pi / 2, a5: 0, d5: 0.057, q5: q5 + pi,
                    alpha6: 0, a6: 0, d6: 0.235, q6: q6 + pi / 2}
    return dh_subs_dict

def dh(alpha, a, d, theta):
    """
    Args:
        alpha: torsion angle
        a: distance
        d: translation
        theta: rotation angle

    Returns: Homogeneous DH matrix

    Important note: use sympy cos/sin arguments instead of math/numpy versions.
    i.e: cos(theta) \ sin(theta)

    """
    # Formula
    return Matrix([[cos(theta), -cos(alpha)*sin(theta), sin(alpha)*sin(theta), a*cos(theta)],
                     [sin(theta), cos(alpha)*cos(theta), -sin(alpha)*cos(theta), a*sin(theta)],
                     [0, sin(alpha), cos(alpha), d],
                     [0, 0, 0, 1]])

def transform_matrices():

    dictionary = set_dh_table()
    T_01 = dh(dictionary[alpha1], dictionary[a1], dictionary[d1], dictionary[q1])
    T_12 = dh(dictionary[alpha2], dictionary[a2], dictionary[d2], dictionary[q2])
    T_23 = dh(dictionary[alpha3], dictionary[a3], dictionary[d3], dictionary[q3])
    T_34 = dh(dictionary[alpha4], dictionary[a4], dictionary[d4], dictionary[q4])
    T_45 = dh(dictionary[alpha5], dictionary[a5], dictionary[d5], dictionary[q5])
    T_56 = dh(dictionary[alpha6], dictionary[a6], dictionary[d6], dictionary[q6])
    Tes = Matrix([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    T = T_01 * T_12 * T_23 * T_34 * T_45 * T_56 * Tes

    return T

def FK(theta_list):

    """
    Args:
        theta_list: joint angle vector ---> list [1,6]
    Returns:
        End effector homogeneous matrix --> Matrix((4, 4))

    Hints:
        - we have added a sympy implementation with missing parts, u dont have to use the same method.
        - chain 'Tes' to T_06 at the end.
    """

    dictionary = set_dh_table()
    T_01 = dh(dictionary[alpha1], dictionary[a1], dictionary[d1], dictionary[q1])
    T_12 = dh(dictionary[alpha2], dictionary[a2], dictionary[d2], dictionary[q2])
    T_23 = dh(dictionary[alpha3], dictionary[a3], dictionary[d3], dictionary[q3])
    T_34 = dh(dictionary[alpha4], dictionary[a4], dictionary[d4], dictionary[q4])
    T_45 = dh(dictionary[alpha5], dictionary[a5], dictionary[d5], dictionary[q5])
    T_56 = dh(dictionary[alpha6], dictionary[a6], dictionary[d6], dictionary[q6])
    Tes = Matrix([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    T = T_01 * T_12 * T_23 * T_34 * T_45 * T_56 * Tes
    # print(T)
    ''' fill angles to dict for sympy calculations'''

    theta_dict = {q1:theta_list[0], q2:theta_list[1], q3:theta_list[2], q4:theta_list[3], q5:theta_list[4], q6:theta_list[5]}
    # for i in range(len(theta_list)):
    #     theta_dict[q[i]] = theta_list[i]
    
    ''' 
    homogeneous transformation matrix from base_link to end_effector [type: numeric matrix] 
    because we are using sympy, we have to use evalf.
    '''
    T_0G_eval = T.evalf(subs=theta_dict, chop=True, maxn=4)

    return T_0G_eval

#
def Jacobian(Q):
    '''

    Args:
        Q: joint configuration list [1,6]

    Returns:
        Full Jacobian matrix [6,6]
    '''

    pass

def LinearJacobian(Q):
    '''

    Args:
        Q: joint configuration list [1,6]

    Returns:
        Linear part of the Jacobian matrix [3,6]
    '''

    pass

def IK_NR_position(guess, target):
    '''

    Args:
        guess: initial angle guess list [1,6] {q1-6}
        target: task configuration tcp position [1,3] {x,y,z}

    Returns:
        Q* - joint configuration angles [1, 6]
    '''
    pass