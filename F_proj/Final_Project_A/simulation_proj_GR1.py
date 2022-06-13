from ast import While
from re import A
from Path_Planner_GR1 import planner, attack_point_calc, goal_point, calc_motor_command
import numpy as np
# from final_proj import *


####################
#### main loop #####
####################
if __name__ == "__main__":



    #### merge with lab code ####

    #### Constants ####
    side = 1  # +x = 1 # -x = -1
    planner_type = 5
    obs = [] # obstacles option

    ### all the units in meters [m]
    ##################


    p_rival, p_own, p_disk = ([0.0, 0.0], [0.02, 0.04],[0.3, 0.4])
    next = []
    phi = []
    command = []
    p_previous = p_disk
    pcur=p_previous
    # going to attac point
    p_attack = attack_point_calc(p_disk, delta = 0.15)
    A_path = planner(p_own, p_attack, obs, planner_type = 5 , B=[-0.05, 0.65] , delta=0.02)  
    s = ([0,0])
    for point in A_path:
        next.append(point - s)
        s = point
        phi.append(np.deg2rad((np.arctan2(next[-1][0], next[-1][1]))))
        command.append(calc_motor_command(phi[-1]))

    p_own = A_path[-1]
    goal_p = goal_point(p_disk, p_own, step_size = 5, side = 1)
    B_path = planner(p_own, goal_p, obs, planner_type = 5 , B=[-0.05, 0.65] , delta=0.02)   

    print(phi)
    print(command)
    print('Goallllllll!!!!!!')