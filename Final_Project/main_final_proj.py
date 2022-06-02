from Final_proj_GR1 import *
from final_proj import *

####################
#### main loop #####
####################
if __name__ == "__main__":

    #### merge with lab code ####

    #### Constants ####
    side = 'Left' # 'Right'
    planner_type = 1
    obs = [] # obstacles option

    ### all the units in meters [m]
    ##################


    p_rival, p_own, p_disk = field_status()

    # steering_angle()       #### Coordinate adjustment ### speak with Osher....

    # attack_point_calc()    #### Calculate an attack location
    p_attack = attack_point_calc(p_disk, delta = 15)


    planner(p_own, p_attack, obs, planner_type , B=[-0.05, 0.65] , delta=0.02)   #### Plane path planner_type=  RRT == 1, RRT* == 2, POTENTIAL == 3, STRAIGHT LINE == 4

    # GoToNextPoint()

    # Check attack position    

    #### final path 1 ####

    # attack orientation

    # push path

    # push

    # Check disk position



    #######################
    #### Goallllllllll ####
    #######################

    ########################
    #### Yala Beitarrrr ####
    ########################


