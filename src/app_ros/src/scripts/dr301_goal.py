#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, time, math
from geometry_msgs.msg import Pose2D

def main_goal():
    global x
    # global y
    # global theta

    rospy.init_node('send_goal', anonymous=True)    # node name
    goal_pub = rospy.Publisher('/goal', Pose2D, queue_size=10, latch=True)

    rate = rospy.Rate(15)

    goal = Pose2D()
    goal.x = x
    #goal.y = y
    #goal.theta = theta
    
    while not rospy.is_shutdown():
        print(goal)
        goal_pub.publish(goal)
        rate.sleep()


################# Main ########################
# x = float(sys.argv[1])
# y = float(sys.argv[2])
# theta = float(sys.argv[3])

x = eval(input("Digite a posicao desejada no eixo X com formato 0.0: "))

lim_x = 10

if x > lim_x:
    x = lim_x
elif x < 0:
    x = 0

if __name__ == '__main__':
    main_goal()
