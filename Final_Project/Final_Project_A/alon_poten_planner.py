import pandas as pd
import numpy as np
import math
import matplotlib.pyplot as plt

def dis(a1, a2):
    a3 = np.linalg.norm(a1 - a2)
    return a3

def plot_path(path, goal_point, obs):


    plt.scatter(path[:,0], path[:,1], color="black", s=5)
    plt.scatter(obs[:,0], obs[:,1], color="g", s=10)
    plt.scatter(goal_point[0], goal_point[1], color="b", s=20)
    plt.scatter(goal_point[0] + 0.15, goal_point[1], color="r", s=25)
    plt.xlim([-0.1, 3])
    plt.ylim([0, 2])
    # plt.pause(0.1)
    plt.show()
    # ax1.plot([0,0], [0,0], [0,160], '-k')
    return

def potential_fun(x_ee, x_goal, obs, delta=0.02):

  
    a = np.array(x_goal) # goal point
    b = np.array(x_ee) # start point

    # obs center
    obs1= np.array([a[0] + 0.15 + 0.1, a[1]]) 
    Ds1 = 0.3 
    obs2= np.array([a[0] + 0.15 , a[1]+ 0.1])  
    Ds2 = 0.2
    obs3=np.array([a[0] + 0.15 , a[1]- 0.1])
    Ds3 = 0.2
    obs4=np.array([a[0] + 0.15 + 0.05 , a[1]- 0.05]) 
    Ds4 = 0.2
    obs5=np.array([a[0] + 0.15 + 0.05 , a[1]+ 0.05])  
    Ds5 = 0.2
    O = np.array([obs1, obs2, obs3, obs4, obs5])

    #### potential constants ####
    c1 = 1 # constant of F_att 0.01
    c2 = 0.001 # constant of F_rep 1e7
    e = 0.02 # error in the end location
    step = 0.17 # step size 
    path = np.array([b]) 
    
    loc = np.copy(b) # Current location

    while dis(a, loc) > e:

        D1 = dis(obs1, loc) # distance from obstacle 1
        if D1 > Ds1:
            Frep1 = np.array([0, 0])
        else: 
            Frep1 = c2 * (1/Ds1 - 1/D1) * (1/D1**2) * (obs1 - loc)
        D2 = dis(obs2, loc) # distance from obstacle 2
        if D2 > Ds2:
            Frep2 = np.array([0, 0])
        else: 
            Frep2 = c2 * (1/Ds2 - 1/D2) * (1/D2**2) * (obs2 - loc)
        D3 = dis(obs3, loc) # distance from obstacle 3
        if D3 > Ds3:
            Frep3 = np.array([0, 0])
        else: 
            Frep3 = c2 * (1/Ds3 - 1/D3) * (1/D3**2) * (obs3 - loc)
        D4 = dis(obs4, loc)
        if D4 > Ds4:
            Frep4 = np.array([0, 0])
        else: 
            Frep4 = c2 * (1/Ds4 - 1/D4) * (1/D4**2) * (obs4 - loc)
        D5 = dis(obs5, loc)
        if D5 > Ds5:
            Frep5 = np.array([0, 0])
        else: 
            Frep5 = c2 * (1/Ds5 - 1/D5) * (1/D5**2) * (obs5 - loc)

        Fatt = c1 * (a - loc)
        F = Fatt + Frep1 + Frep2 + Frep3 + Frep4 + Frep5

        loc = loc + step * F
        
        path = np.append(path, [np.copy(loc)], axis = 0)

    plot_path(path, x_goal,O)
    print(path.shape)
    # print(path)

    return (path)


if __name__ == '__main__':
    obs_ = [0]
    start=[2.5, 0.5]
    goal=[0.7, 0.7]

    pp = potential_fun(start, goal, obs_, delta=0.02)
