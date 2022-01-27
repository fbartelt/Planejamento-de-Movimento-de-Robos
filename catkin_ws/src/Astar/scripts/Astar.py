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
import time

#User defined
theta = 0.001 #Angle about z axis
err = 0.3 #Position tolerance in meters
Kp = 3 #Controller proportional gain
d = 0.1 #For feedback linearization
Vmax = 1000 #Max velocity for the robot
height, width = (16, 16) # in meters
scale = 28 # meters / pixel
robot_pos0 = (-6, 2) # initial position
#robot_pos0 = (-1.46, 2.25) # initial position

pos = gmsg.Point()
pos.x = robot_pos0[0]
pos.y = robot_pos0[1]
pub = None
ranges_ = []
counter = 0
state = 0
states = {0 : 'Calculating A* graph\n...', 1 : 'DONE!\nStarted Moving', 
          2 : 'Reached Goal!!!', 3 : 'Impossible to reach goal'}

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
    ranges_ = data.ranges

def calc_time(total_sec):
    m, s = divmod(total_sec, 60)
    elapsed_time = '{:.2}min {:.2}s'.format(m, s)
    print('(Elapsed time : {})'.format(elapsed_time))


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

def change_state(n):
    global state
    print(f'{states[n]}')
    state = n

class Graph:
    #TODO implement 8-point connectivity
    def __init__(self, pixel, goal, value, g=0, parent=None):
        self.pixel = pixel
        self.goal = goal
        self.parent = parent
        self.neighbors = {'u' : None, #up
                          'l' : None, #left
                          'd' : None, #down
                          'r' : None  #right
                          }
        self.h = self.heuristic()
        self.g = g
        self.obstacle = value
        self.f = self.h + self.g + 1e4 * self.obstacle
    
    def heuristic(self):
        pixel_arr = np.array(self.pixel)
        goal_arr = np.array(self.goal)
        return np.linalg.norm(goal_arr - pixel_arr)

    def add_neighbor(self, loc, npixel, nvalue):
        self.neighbors[loc] = Graph(npixel, self.goal, nvalue, g=self.g+1, parent=self)

class A_star:
    def __init__(self, grid, start, goal):
        self.grid = grid
        self.shape = grid.shape
        self.start = start
        self.goal = goal
        self.graph = Graph(start, goal, grid[start[0], start[1]])
        self.O = [] # priority queue
        self.C = set() # visited nodes
        self.path = None

    def _gen_path(self, node):
        curr = node
        order = []
        while curr:
            order.append(curr.pixel)
            curr = curr.parent
        self.path = order[::-1]
    
    def run(self):
        self.O.append(self.graph)
    
        while self.O:
            ncurr = self.O.pop()

            if ncurr.pixel == self.goal:
                self.O = list(filter(lambda x: x.f < ncurr.g, self.O))
            
            elif ncurr not in self.C:
                self.C.add(ncurr.pixel)
                
                for i, loc in enumerate(ncurr.neighbors.keys()):
                    #move = (-1)**(0 < i < 3) * np.array([0**((i + 1) % 2), 0**(i % 2)]) # x,y
                    move = (-1)**(0 < i < 3) * np.array([-0**(i % 2), 0**((i + 1) % 2)])  # y, x
                    next_pixel = tuple(np.array(ncurr.pixel) + move)
                    O_pxs = [node.pixel for node in self.O]

                    if (next_pixel not in self.C) and (next_pixel not in O_pxs):
                        if ((next_pixel[0] >= 0 and next_pixel[1] >= 0) and 
                            (next_pixel[0] < self.shape[0] and 
                             next_pixel[1] < self.shape[1])):
                                nvalue = self.grid[next_pixel[0], next_pixel[1]]
                                ncurr.add_neighbor(loc, next_pixel, nvalue)
                                self.O.append(ncurr.neighbors[loc])
                
                self.O.sort(key=lambda x : x.f, reverse=True)
                
        self._gen_path(ncurr)
        change_state(1)

def run(x_goal, y_goal, neighbors=4):
    global pub, counter
    print(f'\nGoal set to ({x_goal}, {y_goal}). Using {neighbors} neighbors connectivity.\n')
    twist = gmsg.Twist()
    rospy.init_node('Astar', anonymous=True)
    pub = rospy.Publisher('robot_0/cmd_vel', gmsg.Twist, queue_size=1)
    rospy.Subscriber('robot_0/base_scan', LaserScan, callback_scan)
    rospy.Subscriber('robot_0/base_pose_ground_truth', Odometry, callback_pose)
    rate = rospy.Rate(1000)
    
    curr_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    grid_path = os.path.join(curr_path, 'worlds/grid1.npy')
    grid = np.load(grid_path)
    xgoalp, ygoalp = meters2pixels((x_goal, y_goal))
    x0p, y0p = meters2pixels(robot_pos0)
    planner = A_star(grid, (y0p, x0p), (ygoalp, xgoalp))
    #print((x0p, y0p), (xgoalp, ygoalp))
    change_state(0)
    sttime = time.time()
    planner.run()
    edtime = time.time()
    calc_time(edtime - sttime)
    path = iter(planner.path)
    ynpx, xnpx = next(path)

    x_next, y_next = pixels2meters((xnpx, ynpx))

    while not rospy.is_shutdown():
        if state != 2:
            if np.linalg.norm([pos.x - x_goal, pos.y - y_goal]) < err:
                change_state(2)
            elif np.linalg.norm([pos.x - x_next, pos.y - y_next]) < err:
                ynpx, xnpx = next(path)
                x_next, y_next = pixels2meters((xnpx, ynpx))
            elif state == 3:
                break

            #v, w = traj_controller(x_next, y_next, 1, 1)
            vx = x_next - pos.x
            vy = y_next - pos.y
            v, w = traj_controller(vx, vy)
            twist.linear.x = v
            twist.angular.z = w
            pub.publish(twist)
            counter += 1
            rate.sleep()
            
if __name__ == '__main__':
    try:
        x_goal = float(sys.argv[1])
        y_goal = float(sys.argv[2])
        neighbors = int(sys.argv[3])
        run(x_goal, y_goal, neighbors)
    except rospy.ROSInterruptException:
        pass