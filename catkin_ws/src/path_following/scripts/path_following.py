#!/usr/bin/env python
import rospy
import geometry_msgs.msg as gmsg
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
import math
import numpy as np

#User defined
theta = 0.001 #Angle about z axis
err = 0.5 #Position tolerance in meters
Kp = 2 #Controller proportional gain
d = 0.8 #For feedback linearization
Vmax = 10 #Max velocity for the robot

#Laser params
laser_range_min = 0
laser_range_max = np.inf
laser_step = 0
laser_ang_min = 0
laser_ang_max = 0

pos = gmsg.Point()
pos.x = -5 
pos.y = -5
pub = None
ranges_ = []
counter = 0

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

    if Vtot > Vmax:
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
    global pos, theta
    # Gets current position and orientation (Quaternion)
    pos = data.pose.pose.position
    x_ori = data.pose.pose.orientation.x
    y_ori = data.pose.pose.orientation.y
    z_ori = data.pose.pose.orientation.z
    w_ori = data.pose.pose.orientation.w

    ori = euler_from_quaternion([x_ori, y_ori, z_ori, w_ori])
    theta = ori[2]

def path_gen(k, r, w, step):
    """ Rhodonea curves generator
    """
    t = 0
    while True:
        x = r * np.cos(k * w * t) * np.cos(w * t)
        y = r * np.cos(k * w * t) * np.sin(w * t)
        yield (x, y)
        t = t + step

def run():
    global pub, counter

    twist = gmsg.Twist()
    rospy.init_node('path_follow', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', gmsg.Twist, queue_size=1)
    scan_sub = rospy.Subscriber('/base_scan', LaserScan, callback_scan)
    pos_sub = rospy.Subscriber('/base_pose_ground_truth', Odometry, callback_pose)
    path = path_gen(4, 8, np.pi/5, 1/20)
    rate = rospy.Rate(20)
    x_goal, y_goal = next(path)
    while not rospy.is_shutdown():
        
        if np.linalg.norm([pos.x - x_goal, pos.y - y_goal]) < err:
            x_goal, y_goal = next(path)
        
        v, w = traj_controller(x_goal, y_goal)
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