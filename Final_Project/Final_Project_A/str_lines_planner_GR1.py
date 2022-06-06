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

def strLines_fun(x_ee, x_goal, obs, delta=0.02):

    a = np.array(x_goal) # goal point
    b = np.array(x_ee) # start point
    
    e = 0.03 # error in the end location
    step = 0.5 # step size 
    path = np.array([b]) 
    
    loc = np.copy(b) # Current location
        # obs center
    obs1= np.array([a[0] + 0.15 + 0.1, a[1]]) 
    obs2= np.array([a[0] + 0.15 , a[1]+ 0.05])  
    obs3=np.array([a[0] + 0.15 , a[1]- 0.05])
    obs4=np.array([a[0] + 0.15 + 0.05 , a[1]- 0.05]) 
    obs5=np.array([a[0] + 0.15 + 0.05 , a[1]+ 0.05])  
    O = np.array([obs1, obs2, obs3, obs4, obs5])

    while dis(a, loc) > e:

        if(dis(loc, (a + np.array([0.15, 0]))) < 0.02): # Case 0
            loc = loc + 0.05*(a + np.array([0.15, 0]) + loc) 
        elif(loc[0] <= a[0] + 0.05): # Case 1
            loc = loc + step*(a - loc)
        elif(loc[0] > a[0] + 0.05 and loc[1] > a[1] + 0.1): # Case 2
            t = np.array(a - loc + np.array([0, 0.1]))
            loc = loc + step*t
        elif(loc[0] > a[0] + 0.05 and loc[1] < a[1] - 0.1): # Case 3
            t = np.array(a - loc + np.array([0, -0.1]))
            loc = loc + step*t
        else: # Case 4
            loc = loc + step*np.array([0, 0.2])
        plt.scatter(loc[0], loc[1], color="k", s=5)
        plt.pause(0.1)
        plt.xlim([-0.1, 3])
        plt.ylim([0, 2])
        path = np.append(path, [np.copy(loc)], axis = 0)
       

    plot_path(path, x_goal,O)
    print(path.shape)
    return (path)


if __name__ == '__main__':
    obs_ = [0]
    start=[0.1, 0.1]
    goal=[0.7, 0.7]

    pp = strLines_fun(start, goal, obs_, delta=0.02)