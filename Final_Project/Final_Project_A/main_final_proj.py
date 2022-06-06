import matplotlib.pyplot as plt
import numpy as np
import time
import math
from scipy.spatial.transform import Rotation as R
import sys, os

# Import required packages

from car_control import Controller

from aruco_module import aruco_track
import cv2

from ast import While
from re import A
from Path_Planner import *
from final_proj import *

sys.path.insert(0, r'../common/Aruco_Tracker-master')
sys.path.insert(0, r'../Lab5')

####################
#### main loop #####
####################
if __name__ == "__main__":



    tracker = aruco_track()

    axes_user_ID = 16# int(input('Enter axes user ID:   '))
    car_ID = 3#int(input('Enter car ID:   '))
    opponent_ID =2# int(input('Enter opponent ID:   '))
    disc_ID = 5#int(input('Enter disc ID:   '))
    side =1# int(input('Enter opponent side:   '))
    planner_type =5# int(input('Enter planner_type:   '))
    # side = 1  # +x = 1 # -x = -1
    # planner_type = 1
    cntrlr = Controller(car_ID)  # input car ID
    cntrlr.connect()
    time.sleep(1)
    cntrlr.motor_command(1., 1.)  # Don't move!

    #### Constants ####

    obs = [] # obstacles option
    executed_path = []
    ### all the units in meters [m]
    ##################


    p_rival_mat, p_own_mat, p_disk_mat = field_status() # matrix
    p_rival , p_own , p_disk = calc_xy_base_vector(p_rival_mat, p_own_mat, p_disk_mat) #xy vectors
    # steering_angle()       #### Coordinate adjustment ### speak with Osher....

    # attack_point_calc()    #### Calculate an attack location
    p_previous = p_disk
    pcur=p_previous
    # going to attac point
    goal_achived = True
    while goal_achived is True:
        to_target =True
        while to_target  is True:
            p_attack = attack_point_calc(p_disk, delta = 0.15)
            A_path = planner(p_own, p_attack, obs, planner_type = 5 , B=[-0.05, 0.65] , delta=0.02)   #### Plane path planner_type=  RRT == 1, RRT* == 2, POTENTIAL == 3, STRAIGHT LINE == 4
            for point in A_path:
                GoToNextPoint(p_own, point)
                p_rival_mat, p_own_mat, p_disk_mat = field_status() # matrix
                p_rival , p_own , p_disk = calc_xy_base_vector(p_rival_mat, p_own_mat, p_disk_mat) #xy vectors
                pcur = p_disk
                if static_target(p_previous,pcur)  is False:
                    to_target = True
                    break
                to_target = False
                
        p_rival_mat, p_own_mat, p_disk_mat = field_status() # matrix
        p_rival , p_own , p_disk = calc_xy_base_vector(p_rival_mat, p_own_mat, p_disk_mat) #xy vectors
        goal_p = goal_point(p_disk, p_own, step_size = 5, side = 1)
        B_path = planner(p_own, goal_p, obs, planner_type = 5 , B=[-0.05, 0.65] , delta=0.02)   #### Plane path planner_type=  RRT == 1, RRT* == 2, POTENTIAL == 3, STRAIGHT LINE == 4
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


