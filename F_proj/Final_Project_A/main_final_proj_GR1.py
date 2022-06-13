import matplotlib.pyplot as plt
import numpy as np
import time
import math
from scipy.spatial.transform import Rotation as R
import sys, os

# Import required packages

# sys.path.insert(0, r'../Lab5')
from car_control import Controller
import cv2

from ast import While
from re import A

# Import required packages
from rrt_algo_GR1 import *
from rrt_star_algo_GR1 import *
from poten_planner_GR1 import *
from str_lines_planner_GR1 import *
sys.path.insert(0, r'../common/Aruco_Tracker-master')
from aruco_module import aruco_track


def planner(Pc, Pg, O, planner_type , B=[-0.05, 0.65] , delta=0.02, **args): #B=[-0.05, 0.65]
    """
    Args:
        Pc: start point (x_s, y_s) --> list: len=2 OR np.array(): shape=(2,)
        Pg: end point (x_g, y_g) --> list: len=2 OR np.array(): shape=(2,)
        O: [(x_obs_1, y_obs_2, radius_obs_1), ..., (x_obs_N, y_obs_N, radius_obs_N)
        B: this is the area where you plan the path in. both x and y should be in between these boundaries.
        delta: Path resolution.
        **args: add additional arguments as you please such as whether to plot a path or not.

    Returns:
        path: [[x_1, y_1], [x_2, y_2], ..., [x_M, y_M]] --> List of lists of size 2 (x, y).
                Important: Make sure that your output 'path' is in the right order (from start to goal)
    """
    print("start " + __file__)

    # RRT == 1, RRT* == 2, POTENTIAL == 3, STRAIGHT LINE == 4

    if(planner_type == 1):
    # ====Search Path with RRT====
    # # Set Initial parameters -rrt regular
        rrt = RRT(
            start=Pc, goal=Pg, rand_area=B,
            obstacle_list=O,
            path_resolution=delta,
            # play_area=[0, 10, 0, 14]
            robot_radius=0.005
            )
        path = rrt.planning(animation=True) # True or False

        if path is None:
            print("Cannot find path")
        else:
            print("found path!!")

            # Draw final path
            if show_animation:
                rrt.draw_graph()
                plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
                plt.grid(True)
                plt.pause(0.01)
                # plt.show()
                # save the fig ??


    elif(planner_type == 2):
    # Set Initial parameters - rrt star
        rrt_star = RRTStar(
            start=Pc, goal=Pg, rand_area=B,
            obstacle_list=O,
            expand_dis=0.05,
            robot_radius=0.003)
        path = rrt_star.planning(animation=show_animation)

        if path is None:
            print("Cannot find path")
        else:
            print("found path!!")

            # Draw final path
            if show_animation:
                rrt_star.draw_graph()
                plt.plot([x for (x, y) in path], [y for (x, y) in path], 'r--')
                plt.grid(True)
                plt.pause(0.01)


    elif(planner_type == 3):
        print("potential_field_planning start")
        path = potential_fun(start, goal, obs_, delta=0.02)
        path = np.array(path[::-1])

    elif(planner_type == 4):
        sections = 4 
        path =[]
        xs = (Pg[0] - Pc[0]) / sections
        ys = (Pg[1] - Pc[1]) / sections
        for i in range(sections):
            path.append([[Pc[0] + xs, Pc[1] + ys]])
            Pc[0] = Pc[0] + xs
            Pc[1] = Pc[1] + ys
        path = np.array(path[::-1])
        # path.draw_graph()

    elif(planner_type == 5):
        print("planning path start")
        path =[]
        obs = [0]
        
        path = strLines_fun(Pc, Pg, obs, delta=0.02)
        path = np.array(path[::-1])
    
        
    return path[::-1]

def steering_angle(A_cam_base, A_cam_robot, p_i_base):
    """
    Args:
        A_robot_cam: Homogeneous matrix from car to camera frame
        A_base_cam: Homogeneous matrix from origin to camera frame
        p_i_base: 2d vector of next point in the path with respect to base (origin) frame: (x, y)

    Returns:
        p_i_car: 2d vector of next point in path with respect to car frame: (x, y)
        alpha: Steering angle to next point [degrees].
    """

    A_robot_cam = np.round(np.linalg.inv(A_cam_robot),4)
    A_robot_base = np.round(np.matmul(A_robot_cam, A_cam_base),4)
    p_i_base = np.hstack((np.array(p_i_base), np.array([0,1])))
    p_i_car = np.round(np.dot(A_robot_base, p_i_base),3)

    p_i_car_f = p_i_car[:-2]
    alpha = np.round(np.rad2deg(np.arctan2(p_i_car_f[0], p_i_car_f[1])), 2)
    return (p_i_car_f, alpha)

def attack_point_calc(p_d,delta = 0.10):
    #notice!! side need to be defined as global parameter -1/1
    side = 1
    p_delta = np.array([side*delta,0])
    p_attack = p_d - p_delta
    return p_attack

def goal_point(p_d ,p_own_corrent, step_size, side):

    side = 1
    p_delta = (p_d - p_own_corrent)*step_size*side
    goal_point = p_own_corrent + p_delta
    return goal_point


def static_target(p_previous , pcur ,  max_error = 0.2):
    #notice!! at the end (or start) of the main code we must define the p_previous of the target
    bool_parameter = True
    if abs(pcur[0]-p_previous[0]) > max_error or abs(pcur[1]-p_previous[1]) > max_error:
        bool_parameter = False       
    return bool_parameter

def close_to_disk(p_d,p_o,tol = 0.10):
    d = np.linalg.norm(p_o - p_d)
    if d > tol:
        return False
    else:
        return True

def GoToNextPoint(p_own,point, tolerance = 0.01):
    next_ = [10e2, 10e2]
    while np.linalg.norm(next_[:2]) > tolerance:
        
        try:
            p_rival_mat, p_own_mat, p_disk_mat = field_status() # matrix
            p_rival , p_own , p_disk = calc_xy_base_vector(p_rival_mat, p_own_mat, p_disk_mat)
            next_ = point - p_own #xy in robot coordinate
            phi = np.round(np.deg2rad((np.arctan2(next_[0], next_[1]))), 3)# angle to next point

            # executed_path.append(list(curr))
            print(f' Phi: {round(phi)}, error: {next_[:2]}\n Distance: {np.linalg.norm(next_[:2])}')

        except:
            continue
        if np.linalg.norm(next_) <= tolerance:
            cntrlr.motor_command(1, 1)
            continue
        left, right = calc_motor_command(phi)
        cntrlr.motor_command(left, right)
        ######check#########
        # print(left, right)
        # p_own = point
        ######################

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    print("Reached next point")
    cntrlr.motor_command(1., 1.)

def save(saver):
    logdir_prefix = 'lab-05'

    data_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../Lab5/data')

    if not (os.path.exists(data_path)):
        os.makedirs(data_path)

    logdir = logdir_prefix + '_' + time.strftime("%d-%m-%Y_%H-%M-%S")
    logdir = os.path.join(data_path, logdir)
    if not (os.path.exists(logdir)):
        os.makedirs(logdir)

    print("\n\n\nLOGGING TO: ", logdir, "\n\n\n")

    import pickle
    with open(logdir + '/data' + '.pkl', 'wb') as h:
        pickle.dump(saver, h)

def calc_motor_command(angle):
    '''
    Args:
        angle: steering angle from car pose to target pose

    Returns:
        left and right motor command to align with the steering angle
    '''
    x = angle / 180.
    if x <= 0:
        right = 1.
        left = -2 * x + 1
    else:
        left = 1.
        right = 2 * x + 1
    left = math.copysign(1, left) - left * 0.5
    right = math.copysign(1, right) - right * 0.5
    return -left, -right

def field_status():
    '''
    w.r.t camera frame
    Returns:

    '''
    t_curr, R_curr, ids = tracker.track()
    try:
        t_curr, R_curr, ids = t_curr.squeeze(), R_curr.squeeze(), ids.squeeze()
        trans, rot, homo = {}, {}, {}
        for i in range(len(ids)):
            trans[ids[i]] = t_curr[i, :]
            rot[ids[i]] = R.from_rotvec(R_curr[i, :]).as_matrix()
            homo[ids[i]] = np.vstack((np.hstack((rot[ids[i]], trans[ids[i]].reshape(-1, 1))), np.array([[0, 0, 0, 1]])))
        # Note that everything is with respect to the camera frame!
        p_r = np.linalg.inv(homo[axes_user_ID]) @ homo[car_ID]
        p_o = np.linalg.inv(homo[axes_user_ID]) @ homo[opponent_ID]
        p_d = np.linalg.inv(homo[axes_user_ID]) @ homo[disc_ID]
        return p_r, p_o, p_d

    except:
        print('Error! Cannot detect frames')
        cntrlr.motor_command(1., 1.)

def calc_xy_base_vector(mat1, mat2, mat3):
    p1 = np.array([mat1[0,3],mat1[1,3]])
    p2 = np.array([mat2[0,3],mat2[1,3]])
    p3 = np.array([mat3[0,3],mat3[1,3]])
    return p1, p2, p3



####################
#### main loop #####
####################
if __name__ == "__main__":

    tracker = aruco_track()

    axes_user_ID = 10# int(input('Enter axes user ID:   '))
    car_ID = 2#int(input('Enter car ID:   '))
    opponent_ID =3# int(input('Enter opponent ID:   '))
    disc_ID = 13#int(input('Enter disc ID:   '))
    side =1# int(input('Enter opponent side:   '))
    planner_type =5# int(input('Enter planner_type:   '))
    side = 1  # +x = 1 # -x = -1
    cntrlr = Controller(car_ID)  # input car ID
    cntrlr.connect()
    time.sleep(1)
    cntrlr.motor_command(1., 1.)  # Don't move!

    #### Constants ####

    obs = [] # obstacles option
    executed_path = []
    ### all the units in meters [m]
    ##################

    p_own_mat, p_rival_mat, p_disk_mat = field_status() # matrix
    p_rival , p_own , p_disk = calc_xy_base_vector(p_rival_mat, p_own_mat, p_disk_mat) #xy vectors

    p_previous = p_disk
    pcur=p_previous

    goal_achived = True
    while goal_achived is True:
        to_target =True
        while to_target  is True:
            p_attack = attack_point_calc(p_disk)
            A_path = planner(p_own, p_attack, obs, planner_type , B=[-0.05, 0.65])   #### Plane path planner_type=  RRT == 1, RRT* == 2, POTENTIAL == 3, STRAIGHT LINE == 4
            for point in A_path:
                GoToNextPoint(p_own, point)
                p_rival_mat, p_own_mat, p_disk_mat = field_status() # matrix
                p_rival , p_own , p_disk = calc_xy_base_vector(p_rival_mat, p_own_mat, p_disk_mat) #xy vectors
                p_previous = pcur
                pcur = p_disk
                if static_target(p_previous,pcur)  is False:
                    to_target = True
                    break
                to_target = False
                
        p_rival_mat, p_own_mat, p_disk_mat = field_status() # matrix
        p_rival , p_own , p_disk = calc_xy_base_vector(p_rival_mat, p_own_mat, p_disk_mat) #xy vectors
        goal_p = goal_point(p_disk, p_own, step_size = 5, side = 1)
        B_path = planner(p_own, goal_p, obs, planner_type , B=[-0.05, 0.65])   #### Plane path planner_type=  RRT == 1, RRT* == 2, POTENTIAL == 3, STRAIGHT LINE == 4
        to_goal = True
        while to_goal  is True:
            for point in B_path:
                GoToNextPoint(p_own, point)
                p_rival_mat, p_own_mat, p_disk_mat = field_status() # matrix
                p_rival , p_own , p_disk = calc_xy_base_vector(p_rival_mat, p_own_mat, p_disk_mat) #xy vectors
                if close_to_disk(p_disk, p_own, tol = 0.03)  is False:
                    to_goal = False
                    GoToNextPoint(p_own, (p_own[0] - side*1,p_own[1]))
                    break
                to_goal = False
                goal_achived = False
print('Goallllllll!!!!!!')

    #######################
    #### Goallllllllll ####
    #######################

    ########################
    #### Yala Beitarrrr ####
    ########################


