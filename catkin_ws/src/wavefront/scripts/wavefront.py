#!/usr/bin/env python
import rospy
#import tf_conversions
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
height, width = (16, 16) # in meters
scale = 28 # meters / pixel
robot_size = (0.5, 0.5)
goal = (5, 2)
robot_pos0 = (-6, 2)

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
states = {0 : 'Calculating Wave-Front Planner', 1 : 'Started Moving', 
          2 : 'Reached Goal!!!', 3 : 'Impossible to reach goal'}

def infinite_range(t0):
    """A infinite generator for the <range> function. t0 represents the
    initial value.
    """
    num = t0
    while True:
        yield num
        num += 1

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

def wavefront_planner(grid, goal, neighbors=4):
    """ Generates wavefront grid using 4 or 8 neighbors connectivity.

    Params
    ------
    grid: np.array
        A matrix representing the map already expanded with robot shape
        i.e. the configuration space representation of the map. The
        grid must be binary, where 0s represent free space and 1s
        represent obstacles. Rows must be x-coordinates and columns
        their respective y-coordinates (X x Y) matrix.
    goal: tuple
        The goal coordinates in meters as (x, y) position.
    neighbors: int
        The number of point connectivity to use. Defaults to 4, if
        any other integer is passed, 8 neighbors is assumed.

    Returns
    -------
    wave_grid: np.array
        The wavefront planner grid with weights assigned to each pixel.
    """
    wave_grid = grid.copy()
    #interrupt = 0
    #rx, ry = robot_size
    robot_xp, robot_yp = meters2pixels(robot_pos0)
    goal_xp, goal_yp = meters2pixels(goal)
    #rx = scale * rx
    #ry = scale * ry
    xmax, ymax = grid.shape
    grid[goal_xp, goal_yp] = 2
    rospy.loginfo(f'{states[state]}\n......')
    
    for i in infinite_range(2):
        for pixel in zip(*np.nonzero(wave_grid == i)):
            x, y = pixel
            if neighbors == 4:
                # 4 neighbors
                sub_gridx = wave_grid[np.maximum(0, x - 1) : np.minimum(xmax, x + 2), y]
                sub_gridy = wave_grid[x, np.maximum(0, y - 1) : np.minimum(ymax, y + 2)]
                if (np.array([sub_gridx, sub_gridy]) == 0).any():
                    sub_gridx[sub_gridx == 0] = i+1
                    sub_gridy[sub_gridy == 0] = i+1
                else:
                    change_state(3)
            else:
                # 8 neighbors
                sub_grid = wave_grid[np.maximum(0, x - 1) : np.minimum(xmax, x + 2),
                                     np.maximum(0, y - 1) : np.minimum(ymax, y + 2)]
                if (sub_grid == 0).any():
                    sub_grid[sub_grid == 0] = i+1
                else:
                    change_state(3)

            if x - 1 <= robot_xp <= x + 1 and y - 1 <= robot_yp <= y + 1:
                change_state(1)
                #interrupt = 1
        
        if state:
            rospy.loginfo('DONE!')
            break

    return wave_grid

def path2goal(wave_grid):
    xmax, ymax = wave_grid.shape
    robot_xp, robot_yp = meters2pixels((pos.x, pos.y))
    robot_weight = wave_grid[robot_xp, robot_yp]
    sub_grid = wave_grid[np.maximum(0, robot_xp - 1) : np.minimum(xmax, robot_xp + 2),
                         np.maximum(0, robot_yp - 1) : np.minimum(ymax, robot_yp + 2)]
    idxs = np.argwhere(sub_grid == robot_weight - 1)
    
    if idxs.size > 0:
        pixel2follow = tuple(idxs[0] + [np.maximum(0, robot_xp - 1), 
                                        np.maximum(0, robot_yp - 1)])
        coord2follow = pixels2meters(pixel2follow)
    else:
        change_state(3)

    return coord2follow

def get_laser_params(data):
    """Sets global laser parameters."""
    global laser_range_min, laser_range_max, laser_step, laser_ang_min
    global laser_ang_max
    laser_range_min = data.range_min
    laser_range_max = data.range_max
    laser_step = data.angle_increment
    laser_ang_min = data.angle_min
    laser_ang_max = data.angle_max
    
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
    print(f'{states[n]}')
    state = n

def run(x_goal, y_goal):
    global pub, counter
    twist = gmsg.Twist()
    rospy.init_node('tangent_bug', anonymous=True)
    pub = rospy.Publisher('robot_0/cmd_vel', gmsg.Twist, queue_size=1)
    scan_sub = rospy.Subscriber('robot_0/base_scan', LaserScan, callback_scan)
    pos_sub = rospy.Subscriber('robot_0/base_pose_ground_truth', Odometry, callback_pose, (x_goal, y_goal))
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if np.linalg.norm([pos.x - x_goal, pos.y - y_goal]) < err and state != 2:
            change_state(2)
        
        v, w = boundary_following(x_goal, y_goal)
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
        #rospy.spin()
    except rospy.ROSInterruptException:
        pass