#!/usr/bin/env python
from typing import Counter
import rospy
import tf2_ros as tf2
#import tf_conversions
import sys
import geometry_msgs.msg as gmsg
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import numpy as np

#User defined
theta = 0.001 #Angle about z axis
err = 0.3 #Position tolerance in meters
wstar = 0.8 #Safety distance W*
Kp = 1 #Controller proportional gain
d = 0.8 #For feedback linearization
Vmax = 5 #Max velocity for the robot

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
br = None
ranges_ = []
counter = 0
state = 0
d_rob2goal = 0
d_reach = 0
d_followed = 0
states = {0 : 'Motion to Goal', 1 : 'Boundary Following', 2 : 'Reached Goal'}

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

def find_continuities(ranges):
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
    cont_indx = np.nonzero(np.array(ranges) < laser_range_max)[0]
    # Get superior and inferior limits of continuities
    lim_sup = np.array([x for i, x in enumerate(cont_indx) 
                        if (x + 1 != cont_indx[(i + 1) % len(cont_indx)])])
    lim_inf = np.array([x for i, x in enumerate(cont_indx) 
                        if (x - 1 != cont_indx[(i - 1) % len(cont_indx)])])
    cont_lims = [x for x in zip(lim_inf, lim_sup) if x[0] != x[1]]
    
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
            oi_mat = get_oi_coord(ranges_, list(region))
            oi_inf_dist = np.linalg.norm(oi_mat[0] - np.array([pos.x, pos.y]))
            oi_sup_dist = np.linalg.norm(oi_mat[1] - np.array([pos.x, pos.y]))
            if oi_inf_dist < d_rob2goal or oi_sup_dist < d_rob2goal:
                blocking = True
                reg_num = i
                break

    return blocking, reg_num

def get_oi_coord(ranges, cont_idx_list):
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
        oi_mat[i, :] = range2cart(ranges, int(x))

    return oi_mat

def choose_oi(ranges, cont_idx, x_goal, y_goal):
    """Returns the coordinates of best Oi to follow 

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
        An array with the followed Oi coordinates.
    """
    global d_reach
    pos_vec = np.array([pos.x, pos.y])
    goal_vec = np.array([x_goal, y_goal])

    if isinstance(cont_idx, tuple):
        cont_idx_list = list(cont_idx)
    else:
        cont_idx_list = [x for t in cont_idx for x in t]

    oi_mat = get_oi_coord(ranges, cont_idx_list)
    dist_pos2oi = np.linalg.norm((pos_vec - oi_mat), axis=1)
    dist_oi2goal = np.linalg.norm((goal_vec - oi_mat), axis=1)
    heuristic = dist_pos2oi + dist_oi2goal
    oi2follow = np.argmin(heuristic)
    d_reach = dist_oi2goal[oi2follow]
    oi2follow_coord = oi_mat[oi2follow, :]

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

def callback_scan(data):
    global ranges_
    #rospy.loginfo(str(counter))
    if counter < 10:
        get_laser_params(data)
    ranges_ = data.ranges
    #goal_angle = angle2goal()
    #regions = {
    #    '0' : min(data.ranges[0:143]),
    #    '1' : min(data.ranges[144:287]),
    #    '2' : min(data.ranges[288:431]),
    #    '3' : min(data.ranges[432:575]),
    #    '4' : min(data.ranges[576:719]),
    #    '5' : min(data.ranges[720:863]),
    #    '6' : min(data.ranges[864:1007]),
    #    '7' : min(data.ranges[1008:1151]),
    #    '8' : min(data.ranges[1152:1295]),
    #    '9' : min(data.ranges[1296:1439]),
    #}
    #rospy.loginfo(regions)

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
    global state
    print(f'stage changedd from{state} to {n}')
    state = n

def set_goal(data, args):
    #TODO check if possible to add goal on Stage in runtime
    odom = data
    odom.pose.pose.position.x = args[0]
    odom.pose.pose.position.y = args[1]
    pub_goal = rospy.Publisher('/robot_1/base_pose_ground_truth', Odometry, queue_size=1)
    pub_goal.publish(odom)

def safety_distance(oi_coord):
    """Adds a safety distance to followed Oi.
    """
    oi_dist = np.linalg.norm(oi_coord)
    oi_norm = oi_coord / oi_dist
    oi_safe = (oi_dist + wstar) * oi_norm
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
    global d_followed
    cont_idx = find_continuities(ranges_)
    blocking, reg_num = check_blocking_oi(cont_idx, x_goal, y_goal)
    
    if blocking:
        rospy.loginfo('Blocked by '+str(cont_idx[reg_num]))
        d_reach_old = d_reach
        oi = choose_oi(ranges_, cont_idx[reg_num], x_goal, y_goal)
        #rospy.loginfo('coords: '+str(oi))
        oi_safe = safety_distance(oi)
        #rospy.loginfo('safe: '+str(oi))
        v, w = traj_controller(oi_safe[0], oi_safe[1])
        if np.linalg.norm(([x_goal, y_goal] - oi)) > d_rob2goal:
            d_followed = d_reach_old
            change_state(1)
            rospy.loginfo('Changed to State 1 ' + states[state])
    else:
        rospy.loginfo('FREE')
        v, w = traj_controller(x_goal, y_goal)
    return v, w

def boundary_following():
    global d_followed
    #Derivative of euclidean norm |x| = x/|x|
    cont_lims = find_continuities(ranges_)
    closest_oi = choose_oi(ranges_, cont_lims, pos.x, pos.y)
    D = np.linalg.norm(closest_oi - [pos.x, pos.y])
    G = D - wstar
    DyG = (closest_oi[0] - pos.x) / D
    x_new = pos.x - 1/(DyG) * G
    y_new = pos.y + np.abs(np.sqrt(-pos.x**2 + 2 * pos.x * x_new + wstar**2 
                                   - x_new**2)) * np.sign(closest_oi[1])
    v, w = traj_controller(x_new, y_new)
    if d_reach < d_followed:
        d_followed = d_reach
        change_state(0)
        rospy.loginfo('Changed to State 0 ' + states[state])

    #total_lasers = (laser_ang_max - laser_ang_min) // laser_step
    #right = (-1) ** (last_sensed > total_lasers // 2)
    #margin = right * total_lasers // 8
    #cont_list = [x for t in cont_lims for x in t]
    #for region in cont_lims:
    #    lim_inf, lim_sup = region
    #    if right == 1:
    #        if last_sensed <= lim_inf <= last_sensed + margin:
    #            oi_coord = get_oi_coord()
    #    else:
    #        if last_sensed + margin <= lim_sup <= last_sensed:
    #            oi_coord = get_oi_coord()
    return v, w

def run(x_goal, y_goal):
    global pub, br, counter
    br = tf2.TransformBroadcaster()
    t = gmsg.TransformStamped()
    twist = gmsg.Twist()
    rospy.init_node('tangent_bug', anonymous=True)
    #t.header.stamp = rospy.Time.now()
    #t.header.frame_id = "ZaWarudo"
    #t.child_frame_id = 'robot_0'
    pub = rospy.Publisher('robot_0/cmd_vel', gmsg.Twist, queue_size=1)
    #goal_sub = rospy.Subscriber('robot_1/odom', Odometry, set_goal, (x_goal, y_goal))
    #goal_sub.unregister()
    scan_sub = rospy.Subscriber('robot_0/base_scan', LaserScan, callback_scan)
    pos_sub = rospy.Subscriber('robot_0/base_pose_ground_truth', Odometry, callback_pose, (x_goal, y_goal))
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if np.linalg.norm([pos.x - x_goal, pos.y - y_goal]) < err and state != 2:
            change_state(2)
            rospy.loginfo('!! '+ states[state] +'!!')
        if state == 0:
            v, w = motion2goal(x_goal, y_goal)
            twist.linear.x = v
            twist.angular.z = w
            pub.publish(twist)
        elif state == 1:
            v,w = boundary_following()
            twist.linear.x = v
            twist.angular.z = w
            pub.publish(twist)

        #t.transform.translation = twist.linear
        #t.transform.rotation = quaternion_from_euler(twist.angular.x, twist.angular.y, twist.angular.z)
        counter += 1
        #rospy.loginfo(f'{x_goal}, {y_goal}')
        #br.sendTransform(t)
        rate.sleep()


if __name__ == '__main__':
    try:
        x_goal = float(sys.argv[1])
        y_goal = float(sys.argv[2])
        run(x_goal, y_goal)
        #rospy.spin()
    except rospy.ROSInterruptException:
        pass