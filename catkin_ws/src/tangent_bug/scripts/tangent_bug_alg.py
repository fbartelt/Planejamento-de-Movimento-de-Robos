#!/usr/bin/env python
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

pos = gmsg.Point()
theta = 0.001 # angle about z axis
state = 0
err = 0.3
laser_range_min = 0
laser_range_max = np.inf
laser_step = 0
pub = None
br = None
counter = 0

Kp = 1
d = 0.8
Vmax = 5

def get_laser_params(data):
    """Sets global laser parameters."""
    global laser_range_min, laser_range_max, laser_step
    laser_range_min = data.range_min
    laser_range_max = data.range_max
    laser_step = data.angle_increment

def angle2goal(x_goal, y_goal):
    """Computes the angle to reach the goal in respect to world coords.
    Used to check if an obstacle is obstructing the straight path.
    """
    ang = math.atan2(y_goal - pos.y, x_goal - pos.x) - theta
    # get angle in [-pi, pi)
    ang = np.rad2deg((ang + np.pi) % (2 * np.pi) - np.pi)
    
    return ang

def range2cart(ranges, idx):
    """Returns cartesian coord of Scan measurement in respect to world
    frame.
    """
    x = pos.x + ranges[idx] * np.cos(laser_step * idx + laser_range_min)
    y = pos.y + ranges[idx] * np.sin(laser_step * idx + laser_range_min)
    cart = np.array([x, y])
    
    return cart

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

    for i, region in enumerate(cont_idx):
        lim_inf = laser_step * region[0] - theta
        lim_sup = laser_step * region[1] - theta

        if lim_inf <= ang2g <= lim_sup:
            blocking = True
            reg_num = i
            break
        else:
            blocking = False

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
        oi_mat[i, :] = range2cart(ranges, x)

    return oi_mat

def choose_oi(ranges, cont_idx, x_goal, y_goal):
    #TODO check if its ok
    pos_vec = np.array([pos.x, pos.y])
    goal_vec = np.array([x_goal, y_goal])
    cont_idx_list = [x for t in cont_idx for x in t]
    oi_mat = get_oi_coord(ranges, cont_idx_list)
    dist_pos2oi = np.linalg.norm((pos_vec - oi_mat), axis=1)
    dist_oi2goal = np.linalg.norm((goal_vec - oi_mat), axis=1)
    heuristic = dist_pos2oi + dist_oi2goal
    oi2follow = np.argmin(heuristic)
    
    return oi_mat[oi2follow, :]

    
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
    if counter == 0:
        get_laser_params(data)
    
    goal_angle = angle2goal()
    regions = {
        '0' : min(data.ranges[0:143]),
        '1' : min(data.ranges[144:287]),
        '2' : min(data.ranges[288:431]),
        '3' : min(data.ranges[432:575]),
        '4' : min(data.ranges[576:719]),
        '5' : min(data.ranges[720:863]),
        '6' : min(data.ranges[864:1007]),
        '7' : min(data.ranges[1008:1151]),
        '8' : min(data.ranges[1152:1295]),
        '9' : min(data.ranges[1296:1439]),
    }
    #rospy.loginfo(regions)

def callback_pose(data):
    global pos, theta
    # Gets current position and orientation (Quaternion)
    pos = data.pose.pose.position
    #y_pos = data.pose.pose.position
    x_ori = data.pose.pose.orientation.x
    y_ori = data.pose.pose.orientation.y
    z_ori = data.pose.pose.orientation.z
    w_ori = data.pose.pose.orientation.w

    #t.transform.translation.x = x_pos
    #t.transform.translation.y = y_pos
    #t.transform.translation.z = 0.0
    ori = euler_from_quaternion([x_ori, y_ori, z_ori, w_ori])
    theta = ori[2]

def change_state(n):
    global state
    print(f'stage changedd from{state} to {n}')
    state = n

def rotate(x_goal, y_goal):
    global theta, pub, err, state
    theta_des = math.atan2(y_goal - pos.y, x_goal - pos.x)
    err_theta = theta_des - theta
    twist_msg = gmsg.Twist()
    if math.fabs(err_theta) > math.pi/90:
        twist_msg.angular.z = 0.7 if err_theta > 0 else -0.7
    else:
        change_state(1)
    pub.publish(twist_msg)
    return twist_msg

def motion2goal(x_goal, y_goal):
    global theta, pos, pub, err, state
    theta_des = math.atan2(y_goal - pos.y, x_goal - pos.x)
    err_theta = theta_des - theta
    err_pos = math.sqrt((y_goal - pos.y)**2 + (x_goal - pos.x)**2)

    if err_pos > err:
        twist_msg = gmsg.Twist()
        twist_msg.linear.x = 0.6
        pub.publish(twist_msg)
        return twist_msg
    else:
        change_state(2)
    #if math.fabs(err_theta) > math.pi/90:
    #    change_state(0)

def boundary_following():
    global pub
    twist_msg = gmsg.Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)
    return twist_msg

def set_goal(data, args):
    odom = data
    odom.pose.pose.position.x = args[0]
    odom.pose.pose.position.y = args[1]
    pub_goal = rospy.Publisher('/robot_1/base_pose_ground_truth', Odometry, queue_size=1)
    pub_goal.publish(odom)

def run(x_goal, y_goal):
    global pub, br, counter
    br = tf2.TransformBroadcaster()
    t = gmsg.TransformStamped()
    twist = gmsg.Twist()
    rospy.init_node('tangent_bug', anonymous=True)
    #t.header.stamp = rospy.Time.now()
    t.header.frame_id = "ZaWarudo"
    pub = rospy.Publisher('robot_0/cmd_vel', gmsg.Twist, queue_size=1)
    #goal_sub = rospy.Subscriber('robot_1/odom', Odometry, set_goal, (x_goal, y_goal))
    #goal_sub.unregister()
    scan_sub = rospy.Subscriber('robot_0/base_scan', LaserScan, callback_scan)
    pos_sub = rospy.Subscriber('robot_0/base_pose_ground_truth', Odometry, callback_pose)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        v, w = traj_controller(x_goal, y_goal, 0, 0)
        twist.linear.x = v
        twist.angular.z = w
        pub.publish(twist)
        #if state == 0:
        #    twist = rotate(x_goal, y_goal)
        #elif state == 1:
        #    twist = motion2goal(x_goal, y_goal)
        #else:
        #    twist = boundary_following()
        #t.transform.translation = twist.linear
        #t.transform.rotation = quaternion_from_euler(twist.angular.x, twist.angular.y, twist.angular.z)
        counter += 1
        #rospy.loginfo(f'{x_goal}, {y_goal}')
        #br.sendTransform(t)


if __name__ == '__main__':
    try:
        x_goal = int(sys.argv[1])
        y_goal = int(sys.argv[2])
        run(x_goal, y_goal)
        #rospy.spin()
    except rospy.ROSInterruptException:
        pass