#!/usr/bin/env python
import rospy
import tf2_ros as tf2
import sys
import geometry_msgs.msg as gmsg
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import numpy as np
import pandas as pd

#User defined
theta = 0.001 #Angle about z axis
err = 0.3 #Position tolerance in meters
wstar = 0.8 #Safety distance W*
Kp = 1 #Controller proportional gain
d = 0.8 #For feedback linearization
Vmax = 5 #Max velocity for the robot
zeta = 4
d_star = 1
eta = 5
Q_star = 6

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

def attractive_potential(q, q_goal):
    d_qqgoal = np.linalg.norm(q - q_goal)
    if d_qqgoal <= d_star:
        U_att = 0.5 * zeta * d_qqgoal ** 2
        grad_Uatt = zeta * (q - q_goal)
    else:
        U_att = d_star * zeta * d_qqgoal - 0.5 * zeta * d_star ** 2
        grad_Uatt = d_star * zeta * (q - q_goal) / d_qqgoal
    
    return U_att, grad_Uatt

def repulsive_potential(q):
    U_rep = 0
    grad_Urep = 0
    regions = find_continuities(ranges_)

    for obstacle in regions:
        obs_inf, obs_sub = obstacle
        range_idx_list = np.unique(np.linspace(obs_inf, obs_sub, dtype=np.int64))
        oi_mat = get_oi_coord(range_idx_list)
        d_c = np.linalg.norm((q - oi_mat), axis=1)
        idx_minc = np.argmin(d_c)
        d_qqobs = d_c[idx_minc]
        c = oi_mat[idx_minc, :]
        grad_d = (q - c) / d_c
        
        if d_qqobs <= Q_star:
            U_rep += 0.5 * eta * (1 / d_qqobs - 1 / Q_star)**2
            grad_Urep += eta * (1 / Q_star - 1 / d_qqobs) * 1 / (d_qqobs ** 2) * grad_d
        else:
            U_rep += 0
            grad_Urep += 0 
    
    return U_rep, grad_Urep

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
    print(f'Stage changed from {state} to {n}')
    state = n

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
    pos_sub = rospy.Subscriber('robot_0/base_pose_ground_truth', Odometry, callback_pose)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if np.linalg.norm([pos.x - x_goal, pos.y - y_goal]) < err and state != 2:
            change_state(2)
            rospy.loginfo('Changed to state 2: !! '+ states[state] +' !!')
        if state == 0:
            v, w = motion2goal(x_goal, y_goal)
            twist.linear.x = v
            twist.angular.z = w
            pub.publish(twist)
        elif state == 1:
            v, w = boundary_following(x_goal, y_goal)
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