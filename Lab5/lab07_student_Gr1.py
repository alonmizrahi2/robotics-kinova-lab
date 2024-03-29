import numpy as np
import transformations
from scipy.spatial.transform import Rotation as R
import modern_robotics


def calculate_error(t_curr, R_curr, target_feature_t, target_feature_R, translation_only=False):
    '''
    Calculate error based on the input pose and the target pose
    Input:  (object in current camera frame)
            t_input, 1x3 vector
            R_input, 3x3 matrix
    Output: Error, [t_err, R_err], 6x1
    '''
    t_del = t_curr - target_feature_t
    Rot_Err_Mat = np.dot(R_curr, np.transpose(target_feature_R))
    theta = R.from_matrix(Rot_Err_Mat).as_euler('zyx')
    u = R.from_matrix(Rot_Err_Mat).as_rotvec()
    
    # see paragraph above Eq.13
    # of Chaumette, Francois, and Seth Hutchinson. "Visual servo control. I. Basic approaches."
    # https://hal.inria.fr/inria-00350283/document

    if translation_only:
        error = np.hstack((t_del, np.zeros(3)))
    else:
        error = np.hstack((t_del, theta * u))

    return error

def skew(x):
    return np.array([[0, -x[2], x[1]],
                     [x[2], 0, -x[0]],
                     [-x[1], x[0], 0]])

def feature_jacobian(t_curr, R_curr, target_feature_R):
    '''
    form interaction matrix / feature jacobian base on current camera pose
    Input:  (object in current camera frame)
            t_input, 1x3 vector
            R_input, 3x3 matrix
    Output: Interation Matrix (feature Jacobian), 6x6
    '''
    I = np.identity(3)
    S_Pcur = skew(t_curr)
    Rot_Err_Mat = np.dot(R_curr, np.transpose(target_feature_R))
    theta = R.from_matrix(Rot_Err_Mat).as_euler('zyx')
    u = R.from_matrix(Rot_Err_Mat).as_rotvec()
    j_tu = I - (theta/2) * skew(u) + (1 - (np.sinc(theta) / ((np.sinc(theta / 2)) ** 2))) * np.dot(skew(u), skew(u))
    L_out = np.vstack((np.hstack((-I, S_Pcur)), np.hstack((np.zeros([3,3]), j_tu))))

    return L_out

def control(L, error, _lambda):
    '''
    calculate the twist of camera frame required to reach target pose
    Input:  (object in current camera frame)
            t_input, 1x3 vector
            R_input, 3x3 matrix
    Output: Twist in camera frame
            [nu_c, omg_c], 1x6
    '''
    L_inv = np.linalg.inv(L)
    vel = -_lambda*L_inv*error

    return vel