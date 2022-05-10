# Import required packages
from rrt_algo import *


def planner(Pc, Pg, O, B , delta=0.02, **args): #B=[-0.05, 0.65]
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

    # ====Search Path with RRT====

    # Set Initial parameters
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
            plt.show()
            # save the fig ??----------------------------------

    return path


def steering_angle(A_cam_robot, A_cam_base, p_i_base):
    """
    Args:
        A_robot_cam: Homogeneous matrix from car to camera frame
        A_base_cam: Homogeneous matrix from origin to camera frame
        p_i_base: 2d vector of next point in the path with respect to base (origin) frame: (x, y)

    Returns:
        p_i_car: 2d vector of next point in path with respect to car frame: (x, y)
        alpha: Steering angle to next point [degrees].
    """

    A_robot_cam = np.round(np.linalg.pinv(A_cam_robot),4)
    A_robot_base = np.round(np.matmul(A_robot_cam, A_cam_base),4)
    p_i_base = np.hstack((np.array(p_i_base), np.array([1])))
    p_i_car = np.round(np.matmul(A_robot_base, p_i_base),3)

    p_i_car_f = p_i_car[:-2]
    alpha = np.round(np.rad2deg(np.arctan2(p_i_car_f[0], p_i_car_f[1])), 2)
    return (p_i_car_f, alpha)



if __name__ == '__main__':
    
    # obstacleList = [(0.1, 0.1, 0.04), (0.1, 0.3, 0.04), (0.4, 0.6, 0.07)]  # [x, y, radius]
    # start=[0, 0]
    # goal=[0.09, 0.4]
    # rand_area=[-0.05, 0.65]

    # pp = planner(start, goal, obstacleList, rand_area, delta=0.02)
    # pp2 = np.round(pp[::-1],3)

    # print(pp2)

    # for check
    A_robot_cam = np.array([[-0.1614, -0.6982, 0.6975, 0.412],
                            [0.9769, -0.0127, 0.2133, 0.218],
                            [-0.1400, 0.7158, 0.6841, 0.797],
                            [0., 0., 0., 1.]])
    A_base_cam = np.array([[-0.7537, 0.6208, 0.2157, 0.112],
                           [-0.1910, -0.5292, 0.8246, 0.801],
                           [0.6261, 0.5783, 0.5230, 0.797],
                           [0., 0., 0., 1.]])
    p_i_base = np.array([0.5, 1.1, 0.]) 

    # p_i_car = ([0,0])
    (p_i_car, alpha) = steering_angle(A_robot_cam, A_base_cam, p_i_base)
    
    print(p_i_car, alpha)