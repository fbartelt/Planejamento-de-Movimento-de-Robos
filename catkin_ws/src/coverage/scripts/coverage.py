#!/usr/bin/env python
import rospy
import sys
import geometry_msgs.msg as gmsg
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
import math
import numpy as np
import os
import pickle
from collections import deque

#User defined
theta = 0.001 #Angle about z axis
err = 0.05 #Position tolerance in meters
Kp = 1 #Controller proportional gain
d = 0.1 #For feedback linearization
Vmax = 1000 #Max velocity for the robot
wstar = 1 #Safety distance W*
width, height = (32, 22.5) # in meters
scale = 14 # meters / pixel
#robot_pos0 = (-13.25, 1.25) # initial position
robot_pos0 = (-11.21, 0.7) # initial position

#Laser params
laser_range_min = 0
laser_range_max = 3
laser_step = 0
laser_ang_min = 0
laser_ang_max = 0

pos = gmsg.Point()
pos.x = robot_pos0[0]
pos.y = robot_pos0[1]
pub = None
last_motion_ang = -np.pi / 2
ranges_ = []
counter = 0
state = 1
states = {0 : 'Going back', 1 : 'Moving to next cell', 2 : 'Boundary Following',
          3 : 'Covering', 4 : 'Finished'}

def get_laser_params(data):
    """Sets global laser parameters."""
    global laser_range_min, laser_range_max, laser_step, laser_ang_min
    global laser_ang_max
    laser_range_min = data.range_min
    laser_range_max = data.range_max
    laser_step = data.angle_increment
    laser_ang_min = data.angle_min
    laser_ang_max = data.angle_max

def meters2pixels(coord):
    """Converts coordinate in meters to pixel coordinate.
    """
    xm, ym = coord
    xp = int(np.round(scale * (xm + width // 2)))
    yp = int(np.round(scale * (-ym + height // 2)))
    coord_pixels = (xp, yp)
    
    return coord_pixels

def pixels2meters(coord):
    """Converts pixels coordinate to coordinate in meters.
    """
    xp, yp = coord
    xm = xp / scale - width // 2
    ym = -yp / scale + height // 2
    coord_meters = (xm, ym)
    
    return coord_meters

def traj_controller(vx, vy):
    global Kp, pos, Vmax, theta, d
    u1 = Kp * vx 
    u2 = Kp * vy 
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

def angle2goal(x_goal, y_goal):
    """Computes the angle to reach the goal in respect to world coords.
    Used to check if an obstacle is obstructing the straight path.
    """
    ang = math.atan2(y_goal - pos.y, x_goal - pos.x) - theta
    ang = (ang + np.pi) % (2 * np.pi) - np.pi # get angle in [-pi, pi)
    
    return ang

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
    pos_vec = np.array([pos.x, pos.y])
    goal_vec = np.array([x_goal, y_goal])
    d_rob2goal = np.linalg.norm(pos_vec - goal_vec)

    for i, region in enumerate(cont_idx):
        lim_inf = laser_step * region[0] + laser_ang_min
        lim_sup = laser_step * region[1] + laser_ang_min
        if lim_inf <= ang2g <= lim_sup:
            lim_infp, lim_supp = region
            idxs = np.unique(np.arange(lim_infp, lim_supp, 1, dtype=int))
            oi_mat = get_oi_coord(idxs)
            norm_mat = np.linalg.norm(pos_vec - oi_mat, axis=1)
            oi_inf_dist = np.min(norm_mat)
            oi_sup_dist = np.max(norm_mat)
            margin = 0 * err * (state!=2)
            if oi_inf_dist + margin <= d_rob2goal or oi_sup_dist + margin <= d_rob2goal:
                blocking = True
                reg_num = i
                break

    return blocking, reg_num

def callback_pose(data):
    global pos, theta
    # Gets current position and orientation (Quaternion)
    pos = data.pose.pose.position
    x_ori = data.pose.pose.orientation.x
    y_ori = data.pose.pose.orientation.y
    z_ori = data.pose.pose.orientation.z
    w_ori = data.pose.pose.orientation.w

    ori = euler_from_quaternion([x_ori, y_ori, z_ori, w_ori])
    theta = ori[2]

def change_state(n, verbose=True):
    global state
    if verbose:
        print(f'{states[n]}')
    state = n

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
    
    return cont_lims

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

def get_tangent_vector(region):
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
    region = find_continuities()

    if not region:
        return 0, 0

    if isinstance(region, tuple):
        region = [region]
    for lims in region:
        lim_inf, lim_sup = lims
        idxs = np.unique(np.arange(lim_inf, lim_sup, 1, dtype=int))
        use = np.arange(len(idxs))
        np.random.default_rng().shuffle(use)
        use = sorted(use[:int(0.8*len(use))])
        idxs = idxs[use]
        reg_idx = np.r_[np.array(reg_idx), idxs]

    oi_mat = get_oi_coord(reg_idx)
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
    
    return tangent, closest_point

def safety_distance(oi_coord, tangent_vec):
    """Adds a safety distance to followed Oi.
    """
    vec_robot2oi = oi_coord - [pos.x, pos.y]
    oi_dist = np.linalg.norm(vec_robot2oi)
    if oi_dist == 0:
        oi_dist = 1e-3
    
    oi_norm = vec_robot2oi / oi_dist
    if oi_dist > 1.1*wstar:
        alpha = 1.2
        beta = 1
    else:
        alpha = 3
        beta = 1

    oi_safe = beta * (vec_robot2oi - (wstar * oi_norm)) + alpha * tangent_vec

    return oi_safe

def choose_oi(cont_idx):
    """Returns a velocity vector to closest obstacle point
    Params
    ------
    cont_idx: list
        A list of tuples that define a continuity region by means of
        LaserScan indices.
    
    Returns
    -------
    oi2follow_coord: np.array
        An array with the followed Oi coordinates or velocity vector.
    """
    tangent, closest_point = get_tangent_vector(cont_idx)
    velocity = safety_distance(closest_point, tangent)

    return velocity

def check_nonstationary(initial, points, val=0):
    if points.size == 0:
        return False
    pos_vec = np.array(initial)
    dists = np.linalg.norm(pos_vec - points, axis=1)
    nonstationary = any(dists >= 2)
    if val == 3:
        if state == 2:
            nonstationary = any(dists >= 1)
        else:
            nonstationary = any(dists >= 0.2)

    return nonstationary

def boundary_following():
    cont_lims = find_continuities()

    if not cont_lims:
        change_state(0)
        return 0, 0

    velocity = choose_oi(cont_lims)
    x_new = velocity[0]
    y_new = velocity[1]
    v, w = traj_controller(x_new, y_new)

    return v, w

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
    
    vec = np.array([x_goal, y_goal]) - [pos.x, pos.y]
    v, w = traj_controller(vec[0], vec[1])
    
    return v, w

class Cell:
    def __init__(self, left_corner, right_corner, val, other_lims=[]):
        self.val = val
        self.left_corner = left_corner
        self.right_corner = right_corner
        self.left_neighbors = []
        self.right_neighbors = []
        self.other_lims = other_lims

    def add_neighbor(self, neighbor):
        if neighbor.right_corner > self.right_corner:
            self.right_neighbors.append(neighbor)
            self.right_neighbors.sort(key=lambda x : -x.right_corner[0])
        else:
            self.left_neighbors.append(neighbor)
            self.left_neighbors.sort(key=lambda x : -x.left_corner[0])

def reorder(neighbors, visited):
    n = neighbors.copy()
    nset = set(n)
    intersec = list(nset.intersection(visited))
    
    if intersec:
        uniq_seen = [x for x in n if x not in intersec]
    else:
        uniq_seen = n

    return uniq_seen, intersec

def find_newcell(cell, seen, visited):
    queue = seen.copy()
    next_cell = queue.pop()
    while next_cell in visited:
        next_cell = queue.pop()

    return next_cell

def gen_path2new(cell, goal):
    O = [] # priority queue
    C = set() # visited nodes
    O.append((cell, None, 0))
    T = []
    path = []
 
    while O:
        ncurr, parent, cost = O.pop()

        if ncurr == goal:
            O = list(filter(lambda x: x[2] < cost, O))
        
        elif ncurr not in C:
            C.add(ncurr)
            T.append((ncurr, parent, cost))
            nlist = []
            nlist.extend(ncurr.left_neighbors)
            nlist.extend(ncurr.right_neighbors)

            for neighbor in nlist:
                if (neighbor not in C) and (neighbor not in O):
                    O.append((neighbor, ncurr, cost+1))
            O.sort(key=lambda x : x[2], reverse=True)
    
    while parent:
        path.append(ncurr)
        temp = [x for x in T if x[0] == parent]
        ncurr = temp[0][0]
        parent = temp[0][1]

    return O, C, path

def gen_path(cells, init_pos, total_cells, init_cell=-1):
    uncovered = 1
    visited = []
    seen = []
    counter = 0
    T = []

    if init_cell == -1:
        for cell in cells:
            if (cell.left_corner[0] <= init_pos[0] <= cell.right_corner[0]):
                init_cell = cell
                break

    seen.append(init_cell)
    change_state(1, False)

    while uncovered:
        curr_C = seen.pop()
        test = curr_C
        visited.append(curr_C)
        local_seen = []
        local_vis = []
        if len(set(visited)) == total_cells:
            uncovered = 0
            break
        
        if state:
            if curr_C.left_neighbors:
                l_seen, l_vis = reorder(curr_C.left_neighbors, visited)
                local_vis.extend(l_vis)
                local_seen.extend(l_seen)
            if curr_C.right_neighbors:
                r_seen, r_vis = reorder(curr_C.right_neighbors, visited)
                local_vis.extend(r_vis)
                local_seen.extend(r_seen)

            if not local_seen:
                change_state(0, False)
                if len(local_vis) > 1:
                    test = find_newcell(curr_C, seen, visited)
                    _, _, T = gen_path2new(curr_C, test)
                    local_vis.extend(T)
                    local_seen = T

        if not state and curr_C == test:
            change_state(1, False)
            
        seen.extend(local_vis)
        seen.extend(local_seen)
        
        if counter > 1e6:
            uncovered = 0
        
        counter += 1

    return seen, visited

def run():
    global pub, last_motion_ang
    twist = gmsg.Twist()
    rospy.init_node('Coverage', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', gmsg.Twist, queue_size=1)
    rospy.Subscriber('/base_scan', LaserScan, callback_scan)
    rospy.Subscriber('/base_pose_ground_truth', Odometry, callback_pose)
    rate = rospy.Rate(1000)
    counter = 0
    curr_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    decomposition_path = os.path.join(curr_path, 'worlds/trapz_dec2.pickle')
    
    with open(decomposition_path, 'rb') as handle:
        cells_list = pickle.load(handle)

    x0p, y0p = meters2pixels(robot_pos0)
    _, planned_path = gen_path(cells_list, (x0p, y0p), 10, cells_list[0])
    path = planned_path[::-1]
    print('Cell order:', [x.val for x in path[::-1]])
    input()
    visited = set()
    curr_C = path.pop()
    visited.add(curr_C)
    change_state(1)
    x_next, y_next = pixels2meters(curr_C.left_corner)
    cover_dir = deque(['d', 'r', 'u', 'r'])
    bound_f = ''
    temp = ''
    passed_by = np.array([[]])
    point_from = np.array([pos.x, pos.y])
    all_points = np.array([[]])

    while not rospy.is_shutdown():
        left_corner = np.array(pixels2meters(curr_C.left_corner))
        right_corner = np.array(pixels2meters(curr_C.right_corner))
        cont_idx = find_continuities()
        blocking, reg_num = check_blocking_oi(cont_idx, x_next, y_next)
        nonstat = check_nonstationary(point_from, passed_by, curr_C.val)

        if state == 4:
            v, w = 0, 0

        elif state == 0:
            if np.linalg.norm(left_corner - [pos.x, pos.y]) < 7*err and nonstat:
                if not path:
                    change_state(4)
                else: 
                    curr_C = path.pop()

                    if curr_C in visited:
                        change_state(0)
                    else:
                        visited.add(curr_C)
                        change_state(1)
                    
                    cover_dir = deque(['d', 'r', 'u', 'r'])
                    x_next, y_next = pixels2meters(curr_C.left_corner)
                    point_from = np.array([pos.x, pos.y])
                    passed_by = np.array([[]])

            elif blocking:
                v, w = boundary_following()
            else:
                v, w = motion2goal(x_next, y_next)
            
        elif state == 1:
            if np.linalg.norm(left_corner - [pos.x, pos.y]) < 13*err:
                change_state(2)
                point_from = np.array([pos.x, pos.y])
                passed_by = np.array([[]])
            elif blocking:
                v, w = boundary_following()
            else:
                v, w = motion2goal(x_next, y_next)

        elif state == 2:
            last_motion_ang = -np.pi/2
            
            if np.linalg.norm(left_corner - [pos.x, pos.y]) <= 15*err  and nonstat:
                change_state(3)
                x_next, y_next = np.minimum(pos.x, right_corner[0]), -height / 2
                point_from = np.array([pos.x, pos.y])
                passed_by = np.array([[]])
                bound_f = ''
           
            elif pos.x - right_corner[0] > 0 or bound_f == 'u':
                x_next = right_corner[0] - 0.05
                y_next = height / 2
                cont_idx = find_continuities()
                blocking, reg_num = check_blocking_oi(cont_idx, x_next, y_next)
                if blocking:
                    v, w = boundary_following()
                    bound_f = ''
                else:
                    v, w = motion2goal(x_next, y_next)
                    bound_f = 'u'

            elif pos.x - left_corner[0] < 0 or bound_f == 'd':
                x_next = left_corner[0] + 0.05
                y_next = -height / 2
                cont_idx = find_continuities()
                blocking, reg_num = check_blocking_oi(cont_idx, x_next, y_next)
                if blocking:
                    v, w = boundary_following()
                    bound_f = ''
                else:
                    v, w = motion2goal(x_next, y_next)
                    bound_f = 'd'
            else:
                v, w = boundary_following()
                bound_f = ''

        elif state % 3 == 0:
            last_motion_ang = np.pi/2
            goal = np.array([x_next, y_next])
            
            if np.linalg.norm(goal - [pos.x, pos.y]) <= 5*err:
                cover_dir.rotate(-1)
                temp = ''

            if np.linalg.norm(right_corner - [pos.x, pos.y]) <= 20*err  and nonstat:
                if not path:
                    change_state(4)
                else: 
                    curr_C = path.pop()

                    if curr_C in visited:
                        change_state(0)
                    else:
                        visited.add(curr_C)
                        change_state(1)
                    
                    cover_dir = deque(['d', 'r', 'u', 'r'])
                    x_next, y_next = pixels2meters(curr_C.left_corner)
                    point_from = np.array([pos.x, pos.y])
                    passed_by = np.array([[]])

            elif cover_dir[0] == 'd':
                if temp != 'd':
                    x_next, y_next = np.minimum(pos.x, right_corner[0]), -height / 2
                    temp = 'd'
                else:
                    temp = 'd'
            
            elif cover_dir[0] == 'r':
                if temp != 'r':
                    x_next, y_next = np.minimum(pos.x + 0.5, right_corner[0]), pos.y
                temp = 'r'
    
            elif cover_dir[0] == 'u':
                if temp != 'u':
                    x_next, y_next = np.minimum(pos.x, right_corner[0]), height / 2
                    temp = 'u'
                else:
                    temp = 'u'
            else:
                x_next, y_next = pos.x - 1, pos.y + 0.1

            v, w = motion2goal(x_next, y_next)
            cont_idx = find_continuities()
            blocking, reg_num = check_blocking_oi(cont_idx, x_next, y_next)

            if blocking:
                lim_infp, lim_supp = cont_idx[reg_num]
                idxs = np.unique(np.arange(lim_infp, lim_supp, 1, dtype=int))
                oi_mat = get_oi_coord(idxs)
                pos_vec = np.array([pos.x, pos.y])
                norm_mat = np.linalg.norm(pos_vec - oi_mat, axis=1)
                oi_inf_dist = np.min(norm_mat)
                cover_dir.rotate(-1)
                
                if temp=='r':
                    v, w = boundary_following()
                elif oi_inf_dist <= 0.8: 
                    x_next, y_next = np.minimum(pos.x + 0.5, right_corner[0]), pos.y
                    temp = '' 
                   
        if passed_by.size !=0:
            passed_by = np.r_[passed_by, [[pos.x, pos.y]]]
        else:
            passed_by = np.array([[pos.x, pos.y]])
        
        twist.linear.x = v
        twist.angular.z = w
        pub.publish(twist)
        counter += 1
        rate.sleep()
            
if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass