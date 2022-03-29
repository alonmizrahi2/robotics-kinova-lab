import numpy as np
from sympy import *
from scipy.spatial.transform import Rotation
from lab02_alon_alon_eran import *
alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha1:7')
a1, a2, a3, a4, a5, a6 = symbols('a1:7')
d1, d2, d3, d4, d5, d6 = symbols('d1:7')
q1, q2, q3, q4, q5, q6 = symbols('q1:7')

'''
Hint1: you should use your functions from the previous lab
Hint2: using sympy is easier for debugging, but not mandatory
'''

def Jacobian(Q):
    '''

    Args:
        Q: joint configuration list [1,6]

    Returns:
        Full Jacobian matrix [6,6]
    '''
    T = T_mat()
    Te = T[-1]
    ## calculating vector b_i1, b_i1 =Ri-1*bz
    R01 = T[0][:3,0:3]
    R02 = T[1][:3,0:3]
    R03 = T[2][:3,0:3]
    R04 = T[3][:3,0:3]
    R05 = T[4][:3,0:3]
    bz = Matrix([[0],[0],[1]])
    b_01 = R01 * bz
    b_02 = R02 * bz
    b_03 = R03 * bz
    b_04 = R04 * bz
    b_05 = R05 * bz
    ## calculating the vector r_ie, r_ie = d_0e - d_ie
    d_0e = T[-1][:3, -1]
    d_1e = T[0][:3, -1]
    d_2e = T[1][:3, -1]
    d_3e = T[2][:3, -1]
    d_4e = T[3][:3, -1]
    d_5e = T[4][:3, -1]
    r_1e = d_0e - d_1e
    r_2e = d_0e - d_2e
    r_3e = d_0e - d_3e
    r_4e = d_0e - d_4e
    r_5e = d_0e - d_5e

    Jl = Matrix([[bz.cross(d_0e),b_01.cross(d_1e),b_02.cross(d_2e),b_03.cross(d_3e),b_04.cross(d_4e),b_05.cross(d_5e)]])
    Ja = Matrix([[bz,b_01,b_02,b_03,b_04,b_05]])
    J = Matrix([[Jl],[Ja]])
    return J

def LinearJacobian(Q):
    '''

    Args:
        Q: joint configuration list [1,6]

    Returns:
        Linear part of the Jacobian matrix [3,6]
    '''
    J = Jacobian(Q)
    Jl = J[0:3,:]
    return Jl

def IK_NR_position(guess, target):
    '''

    Args:
        guess: initial angle guess list [1,6] {q1-6}
        target: task configuration tcp position [1,3] {x,y,z}

    Returns:
        Q* - joint configuration angles [1, 6]
    '''
    th0 = guess
    xd = target
    i = 0
    eps = 0.25 ## minimum error
    e = xd - FK(th0)[:3,3].transpose()
    lamb = 1
    while e.norm() > eps:
        Jl = LinearJacobian(th0)
        Jl_pi = Jl.pinv()
        th0 = th0 +lamb*Jl_pi*e

        e = xd - FK(th0)
    Q = th0
    print(e.norm())
    return Q


## main

print([np.deg2rad(0), np.deg2rad(344), np.deg2rad(75), np.deg2rad(0), np.deg2rad(300), np.deg2rad(0)])
print(IK_NR_position([np.deg2rad(5), np.deg2rad(300), np.deg2rad(60), np.deg2rad(4), np.deg2rad(250), np.deg2rad(5)],Matrix([[0.44,0.19,0.45]])))