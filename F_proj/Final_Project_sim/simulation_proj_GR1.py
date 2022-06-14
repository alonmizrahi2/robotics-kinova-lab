from ast import While
from re import A
from Path_Planner_GR1 import planner, attack_point_calc, goal_point, calc_motor_command
import numpy as np
import matplotlib.pyplot as plt


def plot_path(path, push_path, goal_point, goal_final, obs, str):


    plt.scatter(path[:,0], path[:,1], color="black", s=5)
    plt.scatter(push_path[:,0], push_path[:,1], color="b", s=5)
    plt.scatter(obs[:,0], obs[:,1], color="g", s=10)
    plt.scatter(goal_point[0], goal_point[1], color="c", s=20)
    plt.scatter(goal_point[0] + 0.05, goal_point[1], color="r", s=25)
    plt.scatter(goal_final[0], goal_final[1], color="r", s=25)
    plt.plot([0,0], [0,1], "-k")
    plt.plot([1.5,1.5], [0,1], "-k")

    plt.xlim([-0.1, 1.75])
    plt.ylim([0, 1])
    tit = 'Robotic Football Path' + str
    plt.title(tit)
    plt.xlabel('x[m]')
    plt.ylabel('y[m]')
    # plt.legend(['path to attack point','push path','dengerous area','attack point','disk'])
    # plt.pause(0.1)
    plt.show()
    # ax1.plot([0,0], [0,0], [0,160], '-k')
    return

####################
#### main loop #####
####################
if __name__ == "__main__":




    #### Constants ####
    side = 1  # +x = 1 # -x = -1
    planner_type = 3 #### 1 = RRT . 3 = POTENTIAL . 5 = STRAIGHT LINES
    obs = [] # obstacles option

    ### all the units in meters [m]
    ##################


    p_rival, p_own, p_disk = ([0.0, 0.0], [1.2, 0.16],[0.5, 0.15]) #area [x = 0-1.5, y = 0-1]

    a = p_disk


    next = []
    phi = []
    command = []
    p_previous = p_disk
    pcur=p_previous
    # going to attac point
    p_attack = attack_point_calc(p_disk, delta = 0.05)
    A_path = planner(p_own, p_attack, obs, planner_type , B=[-0.05, 0.65] , delta=0.02)  
    A_path = np.vstack((A_path,p_attack))
    s = ([0,0])
    for point in A_path:
        next.append(point - s)
        s = point
        phi.append(np.deg2rad((np.arctan2(next[-1][0], next[-1][1]))))
        command.append(calc_motor_command(phi[-1]))

    p_own = A_path[-1]
    goal_p = goal_point(p_disk, p_own, step_size = 20, side = 1)

    B_path = planner(p_own, goal_p, obs, planner_type, B=[-0.05, 0.65] , delta=0.02)   
    
    b = goal_p
    obs1= np.array([a[0] + 0.1, a[1]]) 
    obs2= np.array([a[0], a[1]+ 0.05])  
    obs3=np.array([a[0], a[1]- 0.05])
    obs4=np.array([a[0] + 0.05 , a[1]- 0.05]) 
    obs5=np.array([a[0] + 0.05 , a[1]+ 0.05])  
    obs6= np.array([b[0] + 0.1, b[1]]) 
    obs7= np.array([b[0] , b[1]+ 0.05])  
    obs8=np.array([b[0] , b[1]- 0.05])
    obs9=np.array([b[0] + 0.05 , b[1]- 0.05]) 
    obs10=np.array([b[0] + 0.05 , b[1]+ 0.05]) 
    O = np.array([obs1, obs2, obs3, obs4, obs5, obs6, obs7, obs8, obs9, obs10])

    B_path = np.array(B_path)
    plot_path(A_path, B_path, p_attack, goal_p, O, '  (Potential)')
    print(p_attack)
    print(A_path)
    # print(phi)
    # print(command)


    
    print('Goallll!!!!!!')