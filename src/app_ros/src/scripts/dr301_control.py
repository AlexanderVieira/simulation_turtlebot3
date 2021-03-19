#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, control_lib
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry

robot_odom = Pose2D()
goal = Pose2D()
K_v = 1
K_omega = 0.8
K_rho = 0.5
K_alpha = 0.5
K_beta = 0.8
gains = [K_v, K_omega,K_rho,K_alpha,K_beta]

def callback_initial_pose(msg):
    global robot_odom    
    robot_odom.x = msg.pose.pose.position.x
    #robot_odom.y = msg.pose.pose.position.y
    #robot_odom.theta = msg.pose.pose.orientation.z

def callback_pose_goal(msg):
    global goal
    goal = msg

def robot_command(robot_odom,goal,gains):
    
    K_v = gains[0]   
    robot_vel = control_lib.linear_control(robot_odom,goal,K_v)
    return robot_vel

def main_control():
    global robot_odom
    global gains
    global goal

    rospy.init_node('dr301_control', anonymous=True)    # node name
    robot_pose = rospy.Subscriber('/odom',Odometry,callback_initial_pose)
    robot_goal = rospy.Subscriber('/goal',Pose2D,callback_pose_goal)
    pub_cmd_vel = rospy.Publisher('/cmd_vel',Twist,queue_size=10)    

    rate = rospy.Rate(15)
    cmd_vel = Twist()
    
    while not rospy.is_shutdown():

        cmd_vel = robot_command(robot_odom,goal,gains) 
        pub_cmd_vel.publish(cmd_vel)
        rate.sleep()

############### Main code ################
if __name__ == '__main__':
    main_control()
