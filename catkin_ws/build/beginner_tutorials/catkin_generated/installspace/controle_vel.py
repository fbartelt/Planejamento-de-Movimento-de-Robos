#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan2 , sqrt

x0 = 0.0
y0 = 0.0
xf = 0.0
yf = 0.0
Theta0 = 0.0
Err = 0.2
Gain = 1.0
d = 0.2
def atribuirvalorpos(data):
    global x0
    global y0
    global Theta0
    x0 = data.x
    y0 = data.y
    Theta0 = data.theta
def mover():
    global xf , yf , Err , Gain , d
    rospy. init_node('controle_vel', anonymous=True)
    pub = rospy.Publisher('turtle1/cmd_vel',Twist , queue_size=1)
    rate = rospy.Rate(1)
    vel_msg = Twist()
    rospy. Subscriber('turtle1/pose', Pose , atribuirvalorpos)
    while not rospy. is_shutdown ():
        xf , yf = input('Digite a coordenada (x,y) desejada : ').split ()
        xf , yf = [ float (xf) for xf in [xf , yf]]
        if xf > 11 or xf < 0:
            rospy.loginfo ('\033[31 mCoordenadas invalidas , digite novamente ...\033[ m')
            continue
        if yf > 11 or yf < 0:
            rospy.loginfo ('\033[31 mCoordenadas invalidas , digite novamente ...\033[ m')
            continue
        while (abs(x0 - xf) > Err or abs(y0 - yf) > Err):
            vel_msg.angular.z = atan2((yf - y0) ,(xf - x0)) - Theta0
            vel_msg.linear.x = Gain * sqrt((xf - x0) ** 2 + ((yf - y0) ** 2))
            if vel_msg.linear.x > 2:
                vel_msg.linear.x = 2
            elif vel_msg.linear.x < -2:
                vel_msg.linear.x = -2
            pub.publish( vel_msg )
            rate.sleep()
if __name__ == '__main__':
    try:
        mover()
    except rospy. ROSInterruptException:
        pass