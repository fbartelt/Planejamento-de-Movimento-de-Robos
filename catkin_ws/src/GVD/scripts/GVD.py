#!/usr/bin/env python
from black import diff
from cairo import Region
from sympy import hessian
from torch import hinge_embedding_loss
import rospy
import tf2_ros as tf2
#import tf_conversions
import sys
import geometry_msgs.msg as gmsg
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import pandas as pd
import pickle
import numpy as np

#User defined
theta = 0.001 #Angle about z axis
err = 0.15 #Position tolerance in meters
wstar = 0.8 #Safety distance W*
Kp = 1 #Controller proportional gain
d = 0.1 #For feedback linearization
Vmax = 1000 #Max velocity for the robot
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
pos.x = np.inf # guarantees no instant end of path planning
pos.y = np.inf
pub = None
ranges_ = []
counter = 0
state = 0
d_rob2goal = 0
d_reach = 0
d_followed = 0
states = {0 : 'Ch', 1 : 'Check Obstacle', 2 : 'Find equidistance', 
          3 : 'Walk GVD', 4: 'dasda'}
tan_list = []
last_motion_vec = np.array([1e-7, 1e-7])
last_motion_ang = np.pi/2
bound_pos = []
check_loop = 0

def get_laser_params(data):
    """Sets global laser parameters."""
    global laser_range_min, laser_range_max, laser_step, laser_ang_min
    global laser_ang_max
    laser_range_min = data.range_min
    laser_range_max = data.range_max
    laser_step = data.angle_increment
    laser_ang_min = data.angle_min
    laser_ang_max = data.angle_max

def change_ang_interval(ang):
    return (ang + np.pi) % (2 * np.pi) - np.pi

def angle2goal(x_goal, y_goal):
    """Computes the angle to reach the goal in respect to world coords.
    Used to check if an obstacle is obstructing the straight path.
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
    Params
    ------
    ranges: sensor_msgs.msg.LaserScan.ranges
        The ranges returned by LaserScan messages.
    
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
    lims_ = list(zip(*cont_lims))
    rep = [x for x in lims_[0] if any((x - 10 <= np.array(lims_[1])) & (np.array(lims_[1]) <= x + 10))]
    rep2 = [x for x in lims_[1] if any((x - 10 <= np.array(rep)) & (np.array(rep) <= x + 10))]
    lim_inf, lim_sup = np.array(lims_[0]), np.array(lims_[1])
    new_inf = [x for x in lim_inf if x not in rep]
    new_sup = [x for x in lim_sup if x not in rep2]
    new_lims = list(zip(new_inf, new_sup))
    print('contlims:', cont_lims)
    print('new lims:', new_lims)
    input('s')
    
    return cont_lims
    
def check_blocking_oi(cont_idx, x_goal, y_goal):
    #TODO check if robot will collide even when goal is visible
    """Checks if any Oi is blocking the path to goal by angle 
    comparison.
    
    Params
    ------
    cont_idx: list
        A list of tuples that define a continuity region by means of
        LaserScan indices.
    x_goal: float
        The horizontal coordinate of the goal
    y_goal: float
        The vertical coordinate of the goal    
    
    Returns
    -------
    blocking: boolean
        Indicates whether an Oi is blocking the path to goal
    reg_num: int or None
        The region number that is blocking the path to goal, i.e. the
        index of <cont_idx> that represents the blocking region. If
        <blocking> == False, this value is None
    """
    ang2g = angle2goal(x_goal, y_goal)
    reg_num = None
    blocking = False

    for i, region in enumerate(cont_idx):
        lim_inf = laser_step * region[0] + laser_ang_min
        lim_sup = laser_step * region[1] + laser_ang_min
        if lim_inf <= ang2g <= lim_sup:
            oi_mat = get_oi_coord(list(region))
            oi_inf_dist = np.linalg.norm(oi_mat[0] - np.array([pos.x, pos.y]))
            oi_sup_dist = np.linalg.norm(oi_mat[1] - np.array([pos.x, pos.y]))
            if oi_inf_dist < d_rob2goal or oi_sup_dist < d_rob2goal:
                blocking = True
                reg_num = i
                break

    return blocking, reg_num

def get_oi_coord(cont_idx_list):
    """Returns a matrix with world cartesian coords of each obstacle.
    Params
    ------
    ranges: sensor_msgs.msg.LaserScan.ranges
        The ranges returned by LaserScan messages.
    cont_idx_list: list
        A list of each <ranges> indices that contains a Oi (endpoint).
    
    Returns
    -------
    oi_mat: np.array
        A matrix that contains each endpoint Oi coords as a row vector,
        i.e. [ [Oi_x1, Oi_x1], [Oi_x2, Oi_x2], ..., [Oi_xn, Oi_xn] ].
    """
    oi_mat = np.empty((len(cont_idx_list), 2))
    for i, x in enumerate(cont_idx_list):
        oi_mat[i, :] = range2cart(ranges_, int(x))

    return oi_mat

def obstacle_tangent_normal(region):
    """Calculates tangent vector based on the smoothed euclidean norm.

    Params
    ------
    region: list or tuple
        A list of tuples or a single tuple of discontinuites.
    
    Returns
    -------
    tangent: np.array
        Tangent vector to the closest obstacle point.
    closest_point
        The closest approximated point.
    """
    reg_idx = []

    if not region:
        return 0, 0

    if isinstance(region, tuple):
        region = [region]
    for lims in region:
        lim_inf, lim_sup = lims
        idxs = np.unique(np.linspace(lim_inf, lim_sup, (lim_sup - lim_inf) + 1, dtype=int))
        reg_idx = np.r_[np.array(reg_idx), idxs]

    oi_mat = get_oi_coord(reg_idx)
    df = pd.DataFrame(oi_mat)
    df.to_csv('/home/fbartelt/Documents/UFMG/Planejamento/logs/GVD.csv')
    #print('SAVED')
    pos_vec = np.array([pos.x, pos.y])
    norm_mat = np.linalg.norm(pos_vec - oi_mat, axis=1) ** 2
    min_idx = np.argmin(norm_mat)
    min_dist = norm_mat[min_idx]
    h = 0.1
    sum_, div_ = np.array([0.0, 0.0]), 0
    
    for i, a in enumerate(norm_mat):
        temp = np.exp((min_dist - a) / (2 * h ** 2))
        sum_ += temp * (pos_vec - oi_mat[i, :])
        div_ += temp
    
    rot90 = np.array([
            [np.cos(last_motion_ang), -np.sin(last_motion_ang)],
            [np.sin(last_motion_ang), np.cos(last_motion_ang)]
        ])
    D = sum_ / div_
    tangent = (rot90 @ D.reshape(-1, 1)).ravel()
    closest_point = pos_vec - D
    
    return tangent, D

def choose_oi(ranges, cont_idx, x_goal, y_goal):
    """Returns the coordinates of best Oi to follow if the robot is in
    motion to goal behavior AND there is no obstacles blocking the goal
    otherwise, it returns a velocity vector
    Params
    ------
    ranges: sensor_msgs.msg.LaserScan.ranges
        The ranges returned by LaserScan messages.
    cont_idx: list
        A list of tuples that define a continuity region by means of
        LaserScan indices.
    x_goal: float
        The horizontal coordinate of the goal.
    y_goal: float
        The vertical coordinate of the goal.
    
    Returns
    -------
    oi2follow_coord: np.array
        An array with the followed Oi coordinates or velocity vector.
    """
    global d_reach, d_followed
    pos_vec = np.array([pos.x, pos.y])
    goal_vec = np.array([x_goal, y_goal])
    
    if isinstance(cont_idx, tuple):
        cont_idx_list = list(cont_idx)
    else:
        cont_idx_list = [x for t in cont_idx for x in t]

    oi_mat = get_oi_coord(cont_idx_list)
    dist_pos2oi = np.linalg.norm((pos_vec - oi_mat), axis=1)
    dist_oi2goal = np.linalg.norm((goal_vec - oi_mat), axis=1)
    heuristic = dist_pos2oi + dist_oi2goal
    
    if state == 1:
        tangent, closest_point = obstacle_tangent_normal(cont_idx)
        safe_oi = safety_distance(closest_point, tangent)
        d_reach = np.linalg.norm(goal_vec - closest_point)
        oi2follow_coord = safe_oi
        
    else:
        tangent, closest_point = obstacle_tangent_normal(cont_idx)
        safe_oi = safety_distance(closest_point, tangent)
        d_reach = np.linalg.norm(goal_vec - closest_point)
        oi2follow_coord = safe_oi

    if d_reach <= d_followed:
        d_followed = d_reach

    return oi2follow_coord

def traj_controller(x_goal, y_goal, vx=0, vy=0):
    global Kp, pos, Vmax, theta, d
    u1 = vx + Kp * (x_goal - pos.x)
    u2 = vy + Kp * (y_goal - pos.y)
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

def traj_controller2(vx, vy):
    """ Same controller, but with velocity inputs.
    """
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

def callback_pose(data, args):
    global pos, theta, d_rob2goal
    # Gets current position and orientation (Quaternion)
    pos = data.pose.pose.position
    x_ori = data.pose.pose.orientation.x
    y_ori = data.pose.pose.orientation.y
    z_ori = data.pose.pose.orientation.z
    w_ori = data.pose.pose.orientation.w

    ori = euler_from_quaternion([x_ori, y_ori, z_ori, w_ori])
    theta = ori[2]
    pos_vec = np.array([pos.x, pos.y])
    goal_vec = np.array([args[0], args[1]])
    d_rob2goal = np.linalg.norm(pos_vec - goal_vec)

def change_state(n):
    global state, check_loop, bound_pos
    state = n
    check_loop = 0
    bound_pos = []
    print(f'{states[state]}')

def safety_distance(oi_coord, tangent_vec):
    """Adds a safety distance to followed Oi.
    """
    vec_robot2oi = oi_coord - [pos.x, pos.y]
    oi_dist = np.linalg.norm(vec_robot2oi)
    if oi_dist == 0:
        oi_dist = 1e-3
    
    oi_norm = vec_robot2oi / oi_dist
    if oi_dist > 1.3*wstar:
        alpha = 1
        beta = 1
    else:
        alpha = 3
        beta = 1

    oi_safe = beta * (vec_robot2oi - (wstar * oi_norm)) + alpha * tangent_vec

    return oi_safe

def motion2goal(x_goal, y_goal):
    """Executes motion to goal behaviour using feedback velocity
    controller with feedback linearization.
    Params
    ------
    x_goal: float
        The horizontal coordinate of the goal.
    y_goal: float
        The vertical coordinate of the goal.
    
    Returns
    -------
    v: float
        The linear velocity given by feedback linearization
    w: float
        The angular velocity given by feedback linearization
    """
    #TODO improve blocking check
    global d_followed, tan_list, last_motion_ang, bound_pos
    cont_idx = find_continuities(ranges_)
    blocking, reg_num = check_blocking_oi(cont_idx, x_goal, y_goal)
    
    if blocking:
        d_reach_old = d_reach
        oi = choose_oi(ranges_, cont_idx[reg_num], x_goal, y_goal)
        oi_safe = oi
        v, w = traj_controller2(oi_safe[0], oi_safe[1])

        if np.linalg.norm(([x_goal, y_goal] - oi_safe)) > d_rob2goal:
            d_followed = d_reach_old
            tan_list = []
            change_state(1)
    else:
        v, w = traj_controller(x_goal, y_goal)
        oi_safe = np.array([x_goal, y_goal])
    
    last_motion_vec = [pos.x, pos.y] - oi_safe
    norm_vec = last_motion_vec / np.linalg.norm(last_motion_vec)
    dot_prod = np.dot(norm_vec, np.array([1, 0]))
    if np.sign(np.arctan(dot_prod) + 2*np.pi) == 1:
        last_motion_ang = np.pi/2
    else:
        last_motion_ang = -np.pi/2
    return v, w

def boundary_following(x_goal, y_goal):
    global d_followed, tan_list, check_loop, bound_pos
    cont_lims = find_continuities(ranges_)

    if not cont_lims:
        change_state(0)
        return 0, 0
    pos_vec = np.array([pos.x, pos.y])
    
    if bound_pos:
        bound_dists = np.linalg.norm(pos_vec - np.array(bound_pos), axis=1)
        
        if not check_loop and np.max(bound_dists) > err:
            check_loop = 1
        elif check_loop and any(bound_dists[:-15] <= err):
            change_state(3)
            return 0, 0

    closest_oi = choose_oi(ranges_, cont_lims, x_goal, y_goal)
    oi = closest_oi
    bound_pos.append([pos.x, pos.y])
    x_new = oi[0]
    y_new = oi[1]
    v, w = traj_controller2(x_new, y_new)
    
    if d_reach <= d_followed - err:
        tan_list = []
        change_state(0)
        bound_pos = np.array([pos.x, pos.y])

    return v, w

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

def repulsive_like(q):
    """
    Params
    ------
    q: np.array
        The robot's current configuration

    Returns
    -------
    """
    regions = find_continuities()
    hessian = np.array([[0.0, 0.0], [0.0, 0.0]])
    D = []

    for obstacle in regions:
        tangent, normal = obstacle_tangent_normal(obstacle)
        distance = np.linalg.norm(normal)
        #print('obs', obstacle, 'normal', normal, 'distance', distance)
        D.append(distance)
        #print('d', D)
    D = np.array(D)
    aux = D[:, None] - D
    diff_D = aux[np.triu_indices(aux.shape[0], k=1)]
    
    return D

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
    hessian = np.array([[0.0, 0.0], [0.0, 0.0]])
    grad_list = []

    for obstacle in regions:
        obs_inf, obs_sub = obstacle
        range_idx_list = np.unique(np.arange(obs_inf, obs_sub, dtype=np.int64))
        qoi_mat = get_oi_config(range_idx_list)
        d_c = np.linalg.norm((q - qoi_mat), axis=1)
        idx_minc = np.argmin(d_c)
        d_qqobs = d_c[idx_minc] - 0.4
        c = qoi_mat[idx_minc, :]
        grad_d = (q - c) / d_qqobs
        den = (((q[1] - c[1])**2 + (q[0] - c[0])**2) ** (3/2))
        dxx = ((q[1] - c[1])**2) / den
        dxy = -((q[1] - c[1]) * (q[0] - c[0])) / den
        dyy = ((q[0] - c[0])**2) / den
        
        #if d_qqobs <= Q_star:
        #eta = 50
        if True:
            U_rep += 0.5 * eta * (1 / d_qqobs - 0 * (1 / Q_star))**2
            grad_Ui = eta * ((1 / Q_star)*0 - 1 / d_qqobs) * (1 / (d_qqobs ** 2)) * grad_d
            grad_list.append(grad_Ui)
            grad_Urep += grad_Ui
            hessian += np.array([[dxx, dxy], [dxy, dyy]])
        else:
            U_rep += 0
            grad_Urep += 0 
    return U_rep, grad_Urep, np.array(grad_list)

def check_repulsive(repulsive_list):
    """ Returns number of repulsive potentials that cancel another.
    Without repetition.
    """
    U_list = repulsive_list.copy()
    
    if isinstance(U_list, list):
        U_list = np.array(U_list)

    diff_U = U_list + U_list[:, None]
    diff_U_norm = np.linalg.norm(diff_U, axis=2)
    norms = diff_U_norm[np.triu_indices(diff_U_norm.shape[0], 1)] # norm between Ui and Uj, i!=j
    equidistant_to = np.sum(norms <= 0.6)
    print('NORMS', norms)

    return equidistant_to

def set_tan_ang(tangent):
    global last_motion_ang
    last_motion_vec = tangent
    norm_vec = last_motion_vec / (np.linalg.norm(last_motion_vec) + 1e-3)
    dot_prod = np.dot(norm_vec, np.array([1, 0]))
    if np.sign(np.arctan(dot_prod) + 2*np.pi) == 1:
        last_motion_ang = np.pi/2
    else:
        last_motion_ang = -np.pi/2
class GVD:
    def __init__(self):
        self.edges = set()
        self.meet_points = set()
    
    def add_edge(self, coord):
        if coord in self.edges:
            self.edges.remove(coord)
            self.meet_points.add(coord)
        else:
            self.edges.add(coord)

def run():
    global pub, counter, d_rob2goal, d_reach, d_followed, bound_pos, path_folowd
    twist = gmsg.Twist()
    rospy.init_node('tangent_bug', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', gmsg.Twist, queue_size=1)
    scan_sub = rospy.Subscriber('/base_scan', LaserScan, callback_scan)
    pos_sub = rospy.Subscriber('/base_pose_ground_truth', Odometry, callback_pose, (0, 0))
    rate = rospy.Rate(1000)
    change_state(0)
    gvd = GVD()
    w = 0
    ref = 0
    rate.sleep()
    tangent = np.array([0.0, 0.0])

    while not rospy.is_shutdown():
        regions = find_continuities()
        print(regions)
        q = np.array([pos.x, pos.y])
        _, grad_Urep, grad_list = repulsive_potential(q)
        diff_D = repulsive_like(q)
        equi2 = check_repulsive(grad_list)
        kkk = 1
        #print(f'Diff D {diff_D}')
        for i, d in enumerate(diff_D[:-1]):
            if any(np.abs(d - diff_D[i+1::]) <= 5e-2):
                kkk += 1
        print(diff_D)
        print(kkk)
        print(f'gradlist: {grad_list}')
        print(f'Cancelling potentials =  {equi2}')
        #input(f'norma: {np.linalg.norm(grad_list, axis=1)}')
        #if kkk >= 3:
        #    print(diff_D)
        #    input(f'kkk 3 maior: {kkk}')
        #    change_state(1)
        #    tangent, normal = obstacle_tangent_normal(regions)

        U_grad = - grad_Urep
        if np.linalg.norm(U_grad) <= 0.6 or state == 1:
            gvd.add_edge((round(pos.x, 1), round(pos.y, 1)))
        if state == 0 and np.linalg.norm(U_grad) <= 0.1:
            change_state(1)
            tangent, normal = obstacle_tangent_normal(regions)
            set_tan_ang(tangent)
            #print('tangent', tangent, np.linalg.norm(tangent))
            U_grad = tangent - grad_Urep
            #input('going tangent ')
        elif state == 1:
            if np.linalg.norm(U_grad) > 0.6:
                change_state(0)
            else:
                U_grad = tangent
        #print('repulsive', U_grad, np.linalg.norm(U_grad))
        #print('hessian', hessian)
        #print('eig', np.linalg.eig(hessian))
        #input('INPUT')
        v, w = traj_controller2(U_grad[0], U_grad[1])
        twist.linear.x = v
        twist.angular.z = w
        pub.publish(twist)
        if counter != 0 and counter % 100 == 0:
            with open('/home/fbartelt/Documents/UFMG/Planejamento/logs/GVD.pickle', 'wb') as handle:
                pickle.dump(gvd, handle, protocol=pickle.HIGHEST_PROTOCOL)
                print('\n'*4+'PICKLEEEEEEEEEEEEEEEE'+'\n'*2)
            input('pickled')

        """ if ranges_ and state == 0:
            min_dist_idx = np.argmin(ranges_)

            if ranges_[min_dist_idx] < laser_range_max:
                min_dist_ang = laser_step * min_dist_idx + laser_ang_min
                w = np.pi + min_dist_ang
                ref = change_ang_interval(w)
                change_state(1)
                print(w, ref)
            else:
                twist.linear.x = 10
                twist.angular.z = 0
                pub.publish(twist)

        elif state == 1:
            #print(np.abs(theta - w), np.deg2rad(5))
            print(ref, theta)
            if np.abs(theta - ref) > np.deg2rad(5):
                twist.linear.x = 0
                twist.angular.z = np.pi / 2
                pub.publish(twist)
            else:
                change_state(2)

        elif state == 2:
            read_front = ranges_[len(ranges_) // 2]
            read_back = ranges_[0]
            diff = (read_front - read_back)
            tangent, normal = obstacle_tangent_normal(regions)
            print(read_front)
            print(tangent, normal)
            input(f'SAVED, diff: {diff}, len_{len(regions)}')

            if len(regions) >= 2:
                if np.abs(diff) <= err:
                    change_state(4)
                    v, w = traj_controller2(tangent[0], tangent[1])
                    twist.linear.x = v
                    twist.angular.z = w
                    pub.publish(twist)
                else:
                    v, w = traj_controller2(normal[0], normal[1])
                    twist.linear.x = v
                    twist.angular.z = w
                    pub.publish(twist)
                    if diff == 0:
                        change_state(0)
        elif state == 3:
            print(ref, theta)
            if np.abs(theta - ref) <= np.deg2rad(5):
                twist.linear.x = 10
                twist.angular.z = 0
                pub.publish(twist)
                change_state(4)
            else:
                twist.linear.x = 0
                twist.angular.z = ref - theta
                pub.publish(twist)
        elif state == 4:
            #input()
            _, normal = obstacle_tangent_normal(regions)
            gvd.add_edge((round(pos.x, 1), round(pos.y, 1)))
            if counter > 100:
                with open('/home/fbartelt/Documents/UFMG/Planejamento/logs/GVD.pickle', 'wb') as handle:
                    pickle.dump(gvd, handle, protocol=pickle.HIGHEST_PROTOCOL)
                    print('\n'*4+'PICKLEEEEEEEEEEEEEEEE'+'\n'*2)
                    input('pickled')

            print(tangent, normal)
            input(f'SAVED, diff: {diff}, len_{len(regions)}')
            print(f'counter {counter}')
            #input()
            a = laser_range_max
            total_lasers = len(ranges_)
            diff = np.abs(ranges_[total_lasers // 4 - 1] - ranges_[int(total_lasers * (3 / 4)) - 1])
            print(diff)
            if len(regions) >= 2:
                if np.abs(diff) <= err:
                    change_state(4)
                    v, w = traj_controller2(tangent[0], tangent[1])
                    twist.linear.x = v
                    twist.angular.z = w
                    pub.publish(twist)
                else:
                    v, w = traj_controller2(normal[0], normal[1])
                    twist.linear.x = v
                    twist.angular.z = w
                    pub.publish(twist)
                    if diff == 0:
                        change_state(0) """

            #if diff > 6*err:
            #    change_state(0)
            #    twist.linear.x = 0
            #    twist.angular.z = 0
            #    pub.publish(twist)
            #else:
            #    twist.linear.x = 10
            #    twist.angular.z = 0
            #    pub.publish(twist)




        #if np.linalg.norm([pos.x - x_goal, pos.y - y_goal]) < err and state != 2:
        #    change_state(2)

        #if state == 0:
        #    v, w = motion2goal(x_goal, y_goal)
        #    twist.linear.x = v
        #    twist.angular.z = w
        #    pub.publish(twist)
        #elif state == 1:
        #    v, w = boundary_following(x_goal, y_goal)
        #    twist.linear.x = v
        #    twist.angular.z = w
        #    pub.publish(twist)
        #else:
        #    ans = input('Do you want to run the algorithm again? (Move the robot with your mouse before answering) [y/N]: ')
        #    if ans.upper() == 'Y':
        #        change_state(0)
        #        counter = 0
        #        d_rob2goal = 0
        #        d_reach = 0
        #        d_followed = 0
        #        bound_pos = []
        #    else:
        #        break

        counter += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        #x_goal = float(sys.argv[1])
        #y_goal = float(sys.argv[2])
        run()
    except rospy.ROSInterruptException:
        pass