#!/usr/bin/env python
import rospy
import sys
import geometry_msgs.msg as gmsg
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
import math
import numpy as np

#User defined
theta = 0.001 #Angle about z axis
err = 1.2 #Position tolerance in meters
Kp = 1 #Controller proportional gain
d = 0.1 #For feedback linearization
Vmax = 20 #Max velocity for the robot
zeta = 5
d_star = 3
eta = 10
Q_star = 1.5
alpha = 0.1

#Laser params
laser_range_min = 0
laser_range_max = np.inf
laser_step = 0
laser_ang_min = 0
laser_ang_max = 0

pos = gmsg.Point()
pos.x = 1000
pos.y = 1000
pub = None
br = None
ranges_ = []
counter = 0
state = 0
states = {0 : '\nMotion to Goal\n', 2 : '\nReached Goal!!!\n'}

def get_laser_params(data):
    """Sets global laser parameters."""
    global laser_range_min, laser_range_max, laser_step, laser_ang_min
    global laser_ang_max
    laser_range_min = data.range_min
    laser_range_max = data.range_max
    laser_step = data.angle_increment
    laser_ang_min = data.angle_min
    laser_ang_max = data.angle_max

def angle2goal(x_goal, y_goal):
    """Computes the angle to reach the goal in respect to world coords.
    """
    ang = math.atan2(y_goal - pos.y, x_goal - pos.x) - theta
    ang = (ang + np.pi) % (2 * np.pi) - np.pi # get angle in [-pi, pi)
    
    return ang

def range2cart(ranges, idx):
    """Returns cartesian coord of Scan measurement in respect to world
    frame.
    """
    T0r = np.array([
                    [np.cos(theta), -np.sin(theta), 0, pos.x],
                    [np.sin(theta), np.cos(theta), 0, pos.y],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]
                   ])
    rangex_robot = ranges[idx] * np.cos(laser_step * idx + laser_ang_min)
    rangey_robot = ranges[idx] * np.sin(laser_step * idx + laser_ang_min)
    range_robot = np.array([rangex_robot, rangey_robot, 1, 1]).reshape(-1, 1)
    range_world = (T0r @ range_robot).ravel()
    
    return range_world[:2]

def find_continuities():
    """ Returns a list of continuities intervals (min, max) for each
    detected obstacle, i.e. the closed intervals of <ranges> indices
    where an obstacle is detected (a continuity).
    
    Returns
    -------
    cont_lims: list
        A list of tuples that represent each continuity interval as
        (min, max), where the values correspond to the <ranges> indices
        where the condition is satisfied.
    """
    # Get indices where laser range < range_max
    cont_indx = np.nonzero(np.array(ranges_) < laser_range_max)[0]
    # Get superior and inferior limits of continuities
    lim_sup = np.array([x for i, x in enumerate(cont_indx) 
                        if (x + 1 != cont_indx[(i + 1) % len(cont_indx)])])
    lim_inf = np.array([x for i, x in enumerate(cont_indx) 
                        if (x - 1 != cont_indx[(i - 1) % len(cont_indx)])])
    cont_lims = [x for x in zip(lim_inf, lim_sup) if x[0] != x[1]]
    
    return cont_lims

def attractive_potential(q, q_goal):
    """Implements a conic + quadratic attractive potential. When the
    robot is more than <d_star> meters from the goal, the conic
    potential is used, otherwise quadratic potential is used.

    Params
    ------
    q: np.array
        The robot's current configuration
    q_goal: np.array
        The goal configuration

    Returns
    -------
    U_att: float
        The attractive potential value
    grad_Uatt: np.array
        The attractive potential gradient
    """
    d_qqgoal = np.linalg.norm(q - q_goal)
    if d_qqgoal <= d_star:
        U_att = 0.5 * zeta * (d_qqgoal ** 2)
        grad_Uatt = zeta * (q - q_goal)
    else:
        U_att = d_star * zeta * d_qqgoal - 0.5 * zeta * (d_star ** 2)
        grad_Uatt = d_star * zeta * (q - q_goal) / d_qqgoal
    
    return U_att, grad_Uatt

def repulsive_potential(q):
    """Implements a repulsive potential as a sum of every repulsive
    potential given by each obstacle in sight. If the robot is more
    than <Q_star> meters from the obstacle, the potential function is
    zero.

    Params
    ------
    q: np.array
        The robot's current configuration

    Returns
    -------
    U_rep: float
        The total repulsive potential value
    grad_Urep: np.array
        The total repulsive potential gradient
    """
    U_rep = 0
    grad_Urep = np.array([0.0, 0.0])
    regions = find_continuities()

    for obstacle in regions:
        obs_inf, obs_sub = obstacle
        range_idx_list = np.unique(np.linspace(obs_inf, obs_sub, dtype=np.int64))
        qoi_mat = get_oi_config(range_idx_list)
        d_c = np.linalg.norm((q - qoi_mat), axis=1)
        idx_minc = np.argmin(d_c)
        d_qqobs = d_c[idx_minc] - 0.4
        c = qoi_mat[idx_minc, :]
        grad_d = (q - c) / d_qqobs
        
        if d_qqobs <= Q_star:
            U_rep += 0.5 * eta * (1 / d_qqobs - 1 / Q_star)**2
            grad_Urep += eta * (1 / Q_star - 1 / d_qqobs) * (1 / (d_qqobs ** 2)) * grad_d
        else:
            U_rep += 0
            grad_Urep += 0 
    
    return U_rep, grad_Urep

def get_oi_config(ranges_idx_list):
    """Returns a matrix with the configuration of each obstacle.

    Params
    ------
    ranges_idx_list: list
        A list of each <ranges> indices that contains a Oi (endpoint).
    
    Returns
    -------
    qoi_mat: np.array
        A matrix that contains each endpoint Oi configuration as a row 
        vector, i.e. 
        [ [Oi1_x, Oi1_y], ..., [Oin_x, Oin_y] ].

    """
    qoi_mat = np.empty((len(ranges_idx_list), 2))
    for i, x in enumerate(ranges_idx_list):
        oi_coord = range2cart(ranges_, int(x))
        qoi_mat[i, :] = oi_coord

    return qoi_mat

def traj_controller(vx, vy):
    global Kp, pos, Vmax, theta, d
    u1 = vx
    u2 = vy
    Vtot = math.sqrt(u1**2 + u2**2)
    if (Vtot > Vmax):
        u1 = u1 * Vmax / Vtot
        u2 = u2 * Vmax / Vtot
    # feddback linearization
    A = [
        [np.cos(theta), -d * np.sin(theta)],
        [np.sin(theta), d * np.cos(theta)]
        ]
    vw = np.linalg.inv(A) @ [[u1], [u2]]
    v = float(vw[0])
    w = float(vw[1])
    
    return v, w

def callback_scan(data):
    global ranges_
    
    if counter < 10:
        get_laser_params(data)
    ranges_ = data.ranges

def callback_pose(data):
    global pos, theta, d_rob2goal
    # Gets current position and orientation (Quaternion)
    pos = data.pose.pose.position
    x_ori = data.pose.pose.orientation.x
    y_ori = data.pose.pose.orientation.y
    z_ori = data.pose.pose.orientation.z
    w_ori = data.pose.pose.orientation.w

    ori = euler_from_quaternion([x_ori, y_ori, z_ori, w_ori])
    theta = ori[2]

def change_state(n):
    global state
    state = n
    print(f'{states[state]}')

def run(x_goal, y_goal):
    global pub, counter
    twist = gmsg.Twist()
    rospy.init_node('potential_function', anonymous=True)
    pub = rospy.Publisher('robot_0/cmd_vel', gmsg.Twist, queue_size=1)
    scan_sub = rospy.Subscriber('robot_0/base_scan', LaserScan, callback_scan)
    pos_sub = rospy.Subscriber('robot_0/base_pose_ground_truth', Odometry, callback_pose)
    rate = rospy.Rate(20)
    U_grad = np.array([100, 100])

    while not rospy.is_shutdown():
        q = np.array([pos.x, pos.y])
        q_goal = np.array([x_goal, y_goal])
        if np.linalg.norm(U_grad) < err and state != 2:
            change_state(2)
            ans = input('Do you want to run the algorithm again? (Move the robot with your mouse before answering) [y/N]: ')
            if ans.upper() == 'Y':
                change_state(0)
                counter = 0
                U_grad = np.array([100, 100])
                q = np.array([pos.x, pos.y])
            else:
                break
        if state == 0:
            _, grad_Uatt = attractive_potential(q, q_goal)
            _, grad_Urep = repulsive_potential(q)
            U_grad = -grad_Uatt - grad_Urep  
            v, w = traj_controller(U_grad[0], U_grad[1])
            twist.linear.x = v
            twist.angular.z = w
            pub.publish(twist)
    
        counter += 1
        rate.sleep()


if __name__ == '__main__':
    try:
        x_goal = float(sys.argv[1])
        y_goal = float(sys.argv[2])
        run(x_goal, y_goal)
    except rospy.ROSInterruptException:
        pass