from re import T
import numpy as np
from sympy import *
from scipy.spatial.transform import Rotation

alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha1:7')
a1, a2, a3, a4, a5, a6 = symbols('a1:7')
d1, d2, d3, d4, d5, d6 = symbols('d1:7')
q1, q2, q3, q4, q5, q6 = symbols('q1:7')


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

    

def xyz_euler(A):
    """
    Extract translation and orientation in euler angles

    Args:
        A: Homogeneous transformation --> np.array((4, 4))

    Returns: x, y, z, thetax, thetay, thetaz --> np.array((1, 6))

    Important note: use numpy arrays

    """
    
    A00 = np.array(A[0, 0]).astype(np.float32)
    A01 = np.array(A[0, 1]).astype(np.float32)
    A02 = np.array(A[0, 2]).astype(np.float32)
    A03 = np.array(A[0, 3]).astype(np.float32)
    A10 = np.array(A[1, 0]).astype(np.float32)
    A11 = np.array(A[1, 1]).astype(np.float32)
    A12 = np.array(A[1, 2]).astype(np.float32)
    A13 = np.array(A[1, 3]).astype(np.float32)
    A20 = np.array(A[2, 0]).astype(np.float32)
    A21 = np.array(A[2, 1]).astype(np.float32)
    A22 = np.array(A[2, 2]).astype(np.float32)
    A23 = np.array(A[2, 3]).astype(np.float32)
    x = A03
    y = A13
    z = A23
    roll = np.rad2deg(np.arctan2(A21, A22)) 
    pitch = np.rad2deg(np.arctan2(-A20, np.sqrt(A21**2 + A22**2))) 
    yaw = np.rad2deg(np.arctan2(A10, A00)) 
    return np.array([x, y, z, roll, pitch, yaw])
    

def angles_to_follow():
    """

    Returns: Dictionary of the desired angels

    """
    angles = { 't1': [np.deg2rad(17.74), np.deg2rad(0.7) ,np.deg2rad(94.11),np.deg2rad(17.81), np.deg2rad(-10), np.deg2rad(0)],
               't2': [np.deg2rad(0), np.deg2rad(0), np.deg2rad(0), np.deg2rad(0), np.deg2rad(0),np.deg2rad(0)],
               't3': [np.deg2rad(356), np.deg2rad(21), np.deg2rad(150), np.deg2rad(272),np.deg2rad(320) ,np.deg2rad(273) ],
               't4': [np.deg2rad(270),np.deg2rad(148) , np.deg2rad(148), np.deg2rad(270), np.deg2rad(140),np.deg2rad(0) ],
               't5': [np.deg2rad(308), np.deg2rad(88),np.deg2rad(86) ,np.deg2rad(38) , np.deg2rad(358),np.deg2rad(358)],
               't6': [np.deg2rad(335), np.deg2rad(344),np.deg2rad(99) ,np.deg2rad(335) , np.deg2rad(336),np.deg2rad(233)],
               't7': [np.deg2rad(20), np.deg2rad(333),np.deg2rad(6) ,np.deg2rad(244) , np.deg2rad(284),np.deg2rad(322)]}

    return angles

#  { 't1': [np.deg2rad(0), np.deg2rad(344) ,np.deg2rad(75),np.deg2rad(0), np.deg2rad(300), np.deg2rad(0)],
#                't2': [np.deg2rad(0), np.deg2rad(0), np.deg2rad(0), np.deg2rad(0), np.deg2rad(0),np.deg2rad(0)],
#                't3': [np.deg2rad(356), np.deg2rad(21), np.deg2rad(150), np.deg2rad(272),np.deg2rad(320) ,np.deg2rad(273) ],
#                't4': [np.deg2rad(270),np.deg2rad(148) , np.deg2rad(148), np.deg2rad(270), np.deg2rad(140),np.deg2rad(0) ],
#                't5': [np.deg2rad(308), np.deg2rad(88),np.deg2rad(86) ,np.deg2rad(38) , np.deg2rad(358),np.deg2rad(358)],
#                't6': [np.deg2rad(335), np.deg2rad(344),np.deg2rad(99) ,np.deg2rad(335) , np.deg2rad(336),np.deg2rad(233)],
#                't7': [np.deg2rad(20), np.deg2rad(333),np.deg2rad(6) ,np.deg2rad(244) , np.deg2rad(284),np.deg2rad(322)],
#                't8': [np.deg2rad(266), np.deg2rad(312),np.deg2rad(350) ,np.deg2rad(337) , np.deg2rad(252),np.deg2rad(265)],
#                't9': [np.deg2rad(255), np.deg2rad(273),np.deg2rad(284) ,np.deg2rad(357) , np.deg2rad(237),np.deg2rad(248)],
#                't10': [np.deg2rad(218), np.deg2rad(245),np.deg2rad(218) ,np.deg2rad(58) , np.deg2rad(266),np.deg2rad(318)],# [radians]
#                 't11':[np.deg2rad(295), np.deg2rad(39) ,np.deg2rad(77),np.deg2rad(280), np.deg2rad(220), np.deg2rad(30)]}# [np.deg2rad(295), np.deg2rad(39) ,np.deg2rad(77),np.deg2rad(280), np.deg2rad(220), np.deg2rad(210)]} # part b
#####

np.set_printoptions(suppress=True)
ang = angles_to_follow()
theta_list = [ang['t1'],ang['t2'],ang['t3'],ang['t4'],ang['t5'],ang['t6'],ang['t7']]#,ang['t8'],ang['t9'],ang['t10'],ang['t11']]

# part a
for i in range(7):

    A1=FK(theta_list[i])
    a1 = np.array(A1).astype(np.float32)
    a1 = np.round(a1,3)
    a1 = Array(a1)
    print(f"Actuators Angles {i+1}")
    print(np.rad2deg(theta_list[i]))
    print(f"Homogeneous Matrices {i+1}")
    pprint(a1)
    print(f"Position and Euler Angles {i+1}")
    v1 = xyz_euler(A1)
    v1 = np.round(v1, 2)
    print(v1)

# part b

# A2 =FK(ang['t11'])
# print("Actuators Angles")
# print(np.rad2deg(theta_list[10]))
# print("Homogeneous Matrices T_ee")
# a2 = np.array(A2).astype(np.float32)
# a2 = np.round(a2,3)
# a2 = Array(a2)
# pprint(a2)
# print("Homogeneous Matrices T_ee*")
# Tee1 = ([0,1,0,0.41],[1,0,0,0],[0,0,-1,0],[0,0,0,1])
# Tee1 = Array(Tee1)
# pprint(Tee1)
# print("Position and Euler Angles")
# v2 = xyz_euler(A2)
# v2 = np.round(v2, 2)
# print(v2)
# print("Position and Euler Angles *")
# v3 = ([0.41, 0, 0, 180, 0, 90])
# print(v3)

# error
# err1 = np.dot(Tee1, a2)
# mse = (np.square(Tee1 - a2)).mean(axis=None)
# mse = round(mse,4)
# print("Mean Squere Error")
# print(f'{mse*100}[%]')
# Mean Squere Error
# 2.54[%]