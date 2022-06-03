from ast import While
from re import A
from Path_Planner import *
from final_proj import *


####################
#### main loop #####
####################
if __name__ == "__main__":

    #### merge with lab code ####

    #### Constants ####
    side = 1  # +x = 1 # -x = -1
    planner_type = 1
    obs = [] # obstacles option

    ### all the units in meters [m]
    ##################


    p_rival, p_own, p_disk = field_status()

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
                GoToNextPoint(point)
                p_rival, p_own, p_disk = field_status()
                pcur = p_disk
                if static_target(p_previous,pcur)  is False:
                    to_target = True
                    break
                to_target = False
                
        p_rival, p_own, p_disk = field_status()
        goal_p = goal_point(p_disk, p_own, step_size = 5, side = 1)
        B_path = planner(p_own, goal_p, obs, planner_type = 5 , B=[-0.05, 0.65] , delta=0.02)   #### Plane path planner_type=  RRT == 1, RRT* == 2, POTENTIAL == 3, STRAIGHT LINE == 4
        to_goal = True
        while to_goal  is True:
            for point in B_path:
                GoToNextPoint(point)
                p_rival, p_own, p_disk = field_status()
                if close_to_disk(p_disk, p_own, tol)  is False:
                    to_goal = False
                    GoToNextPoint((p_own[0] - side*1,p_own[1]))
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


