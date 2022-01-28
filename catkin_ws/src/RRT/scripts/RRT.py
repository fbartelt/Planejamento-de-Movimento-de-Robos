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
from collections import deque

#User defined
theta = 0.001 #Angle about z axis
err = 0.15 #Position tolerance in meters
Kp = 3 #Controller proportional gain
d = 0.1 #For feedback linearization
Vmax = 1000 #Max velocity for the robot
height, width = (16, 16) # in meters
scale = 28 # meters / pixel
robot_pos0 = (-6, 2) # initial position
#robot_pos0 = (-1.46, 2.25)

pos = gmsg.Point()
pos.x = robot_pos0[0]
pos.y = robot_pos0[1]
pub = None
ranges_ = []
state = 0
states = {0 : 'Generating RRT path\n...', 
          1 : 'DONE!\nStarted Moving', 
          2 : 'Reached Goal!!!', 
          3 : 'Impossible to reach goal or small <max_iter>'}

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

class Node:
    def __init__(self, pixel, parent=None):
        self.children = []
        self.pixel = pixel
        self.pos = pixels2meters(self.pixel)
        self.parent = parent

    def add_children(self, child):
        self.children.append(child)

class RRT:
    """ RRT tree based on a premade grid

    Attributes:
    -----------
    root: Node
        The root node.
    grid: np.array
        Grid of the map as a binary matrix (1=Obstacle, 0=Free space).
    path: list
        Generated path to goal as a stack, i.e. reversed follow order.
    max_iter: int
        Max number of iterations for valid random configuration
        generation.
    
    Methods:
    --------
    gen_qrand(qgoal)
        Generates random configuration in free space with uniform 
        distribution.
    find_nearest_q(qrand)
        Finds the nearest configuration in the tree to the random 
        configuration.
    find_next_q(nearest_q, qrand)
        Adds a new node that is 2 meters away from nearest_q in the
        direction of qrand.
    check_collision(nearest_px, next_px)
        Checks if path from nearest_px to next_px is collision-free.
    merge(qnew, tree)
        Merges both trees if there's a path connecting them.
    """

    def __init__(self, root, grid, max_iter=1000):
        self.root = root
        self.grid = grid
        self.path = None
        self.max_iter = max_iter
    
    def gen_qrand(self, qgoal):
        """ Generates random configuration in free space with uniform 
        distribution.

        Parameters:
        ----------
        qgoal: Node
            Goal node of current tree (qf if Tinit and q0 if Tgoal)

        Returns:
        --------
        qrand: np.array
            Position of the random configuration in free space. If
            after max_iter iterations qrand is not in free space, None
            is returned.
        """
        free, counter = 0, 0
        rng = np.random.default_rng()
        qrand = None
        x_inf, y_inf = 0, 0
        x_sup, y_sup = self.grid.shape[0] - 1, self.grid.shape[1] - 1
        
        xoptions = np.unique(np.linspace(x_inf, x_sup, dtype=np.int32))
        yoptions = np.unique(np.linspace(y_inf, y_sup, dtype=np.int32))

        while not free and counter <= self.max_iter:
            xrand = rng.choice(xoptions)
            yrand = rng.choice(yoptions)
            qrand = np.array([xrand, yrand])
            if self.grid[yrand, xrand] == 0:
                free = 1
            counter += 1

        return qrand

    def find_nearest_q(self, qrand):
        """ Finds the nearest configuration in the tree to the random 
        configuration.

        Parameters:
        ----------
        qrand: np.array
            Position of the random configuration in free space.
            Generated by gen_qrand()

        Returns:
        --------
        nearest_q: Node
            The node in current tree that is the nearest to qrand
        """
        stack = self.root.children.copy()
        nearest_q = self.root
        qrand_pos = np.array(pixels2meters(qrand))
        min_dist = np.linalg.norm(qrand_pos - np.array(nearest_q.pos))

        while stack:
            node = stack.pop()
            x, y = node.pos
            curr_dist = np.linalg.norm(qrand_pos - [x, y])
            stack.extend(node.children.copy())
            
            if curr_dist <= min_dist:
                nearest_q = node
                min_dist = curr_dist
        
        return nearest_q
    
    def find_next_q(self, nearest_q, qrand):
        """ Adds a new node that is 2 meters away from nearest_q in the
        direction of qrand.

        Parameters:
        ----------
        nearest_q: Node
            The node in current tree that is the nearest to qrand.
            Returned by find_nearest_q
        qrand: np.array
            Position of the random configuration in free space.
            Generated by gen_qrand()

        Returns:
        --------
        next_q: Node
            A new node 2 meters away from nearest_q in direction of
            qrand.
        """
        xrand, yrand = pixels2meters(qrand)
        near_pos = np.array(nearest_q.pos)
        next_vec = [xrand, yrand] - near_pos
        norm_vec = next_vec / (np.linalg.norm(next_vec) + 1e-3)
        next_qpx = meters2pixels(near_pos + 2 * norm_vec)
        next_q = None
        collision = self.check_collision(nearest_q.pixel, next_qpx)
        
        if not collision:
            next_q = Node(next_qpx, nearest_q)
        
        return next_q
    
    def check_collision(self, nearest_px, next_px):
        """ Checks if path from nearest_px to next_px is collision-free.

        Parameters:
        ----------
        nearest_px: tuple
            Pixel coordinates of nearest_q (returned by find_nearest_q())
        next_px: tuple
            Pixel coordinates of next_q (returned by find_next_q())

        Returns:
        --------
        collision: int (bool)
            0 if there is no collision in path nearest_q -> next_q
            1 if there is no collision-free path.
        """
        collision = 0
        xnxt_px, ynxt_px = pixels2meters(next_px)
        xnst_px, ynst_px = pixels2meters(nearest_px)
        x = np.array([xnst_px, xnxt_px])
        y = np.array([ynst_px, ynxt_px])
        
        if next_px[0] == nearest_px[0]:
            x, y = y, x
            xg = np.linspace(ynst_px, ynxt_px)
            A = np.vstack([x, np.ones(len(x))]).T
            m, c = np.linalg.lstsq(A, y, rcond=None)[0]
            yg = m * xg + c
            xg, yg = yg, xg
        else:
            xg = np.linspace(xnst_px, xnxt_px)
            A = np.vstack([x, np.ones(len(x))]).T
            m, c = np.linalg.lstsq(A, y, rcond=None)[0]

            yg = m * xg + c
        
        pixels = [meters2pixels(p) for p in list(zip(xg, yg))]

        for pixel in pixels:
            if self.grid[int(pixel[1]), int(pixel[0])] == 1:
                collision = 1
                break
            
        return collision

    def merge(self, qnew, tree):
        """ Merges both trees if there's a path connecting them. 
        Merging implies that self.path is updated. tree.path is also
        updated and it is a reversed version of self.path. Both paths
        are stacks, such that the correct order to reach tree.root from
        self.root is self.path[-1], self.path[-2], ..., self.path[0].

        Parameters:
        ----------
        qnew: Node
            Last node generated by find_next_q().
        tree: RRT
            The other RRT (e.g. Tgoal if self == Tinit).

        Returns:
        --------
        collision: int (bool)
            0 if there is no collision in path nearest_q -> next_q
            1 if there is no collision-free path.
        """
        nearest_q = tree.find_nearest_q(qnew.pixel)
        path = []
        collision = tree.check_collision(nearest_q.pixel, qnew.pixel)
        if not collision:
            
            for node in [nearest_q, qnew]:
                curr = node
                while curr:
                    path.append(curr.pixel)
                    curr = curr.parent
                path = path[::-1]

            self.path = path[::-1]
            tree.path = path
            
        return collision

def run_RRT(Tinit, Tgoal, max_iter=15000):
    """ Executes RRT planner.

    Parameters:
    ----------
    Tinit: RRT
        Initial position RRT.
    Tgoal: RRT
        Goal RRT.
    max_iter: int
        Max iterations to try generating a path from initial
        configuration to goal configuration.
    """
    trees = deque([Tinit, Tgoal])
    counter, disc = 0, 1

    while disc and counter <= max_iter:
        tree = trees[0]
        qrand = tree.gen_qrand(trees[1].root)
        if qrand is not None:
            qnearest = tree.find_nearest_q(qrand)
            qnext = tree.find_next_q(qnearest, qrand)

            if qnext:
                qnearest.add_children(qnext)
                disc = tree.merge(qnext, trees[1])
            
        counter += 1
        trees.rotate()
    
    if disc:
        print('MAX ITERATION REACHED: ', counter)
        change_state(3)
    else:
        change_state(1)


def run(x_goal, y_goal):
    global pub
    print(f'\nGoal set to ({x_goal}, {y_goal}).\n')
    twist = gmsg.Twist()
    rospy.init_node('RRT', anonymous=True)
    pub = rospy.Publisher('robot_0/cmd_vel', gmsg.Twist, queue_size=1)
    rospy.Subscriber('robot_0/base_scan', LaserScan, callback_scan)
    rospy.Subscriber('robot_0/base_pose_ground_truth', Odometry, callback_pose)
    rate = rospy.Rate(1000)
    
    curr_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    grid_path = os.path.join(curr_path, 'worlds/grid1.npy')
    grid = np.load(grid_path)
    xgoalp, ygoalp = meters2pixels((x_goal, y_goal))
    x0p, y0p = meters2pixels(robot_pos0)
    q0 = Node((x0p, y0p))
    qf = Node((xgoalp, ygoalp))
    Tinit = RRT(q0, grid)
    Tgoal = RRT(qf, grid)
    change_state(0)
    sttime = time.time()
    run_RRT(Tinit, Tgoal)
    edtime = time.time()
    calc_time(edtime - sttime)
    path = Tinit.path.copy()
    xnpx, ynpx = path.pop()

    x_next, y_next = pixels2meters((xnpx, ynpx))
    print('Next configuration', (x_next, y_next))

    while not rospy.is_shutdown():
        if state == 1:
            if np.linalg.norm([pos.x - x_next, pos.y - y_next]) < err:
                if path:
                    xnpx, ynpx = path.pop()
                    x_next, y_next = pixels2meters((xnpx, ynpx))
                    print('Next configuration', (x_next, y_next))
                else:
                    change_state(2)

            vx = x_next - pos.x
            vy = y_next - pos.y
            v, w = traj_controller(vx, vy)
            twist.linear.x = v
            twist.angular.z = w
            pub.publish(twist)
            rate.sleep()
            
if __name__ == '__main__':
    try:
        x_goal = float(sys.argv[1])
        y_goal = float(sys.argv[2])
        run(x_goal, y_goal)
    except rospy.ROSInterruptException:
        pass