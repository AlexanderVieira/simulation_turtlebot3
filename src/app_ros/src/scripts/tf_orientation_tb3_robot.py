#!/usr/bin/env python3

#this is a simple program that subscribes to the odom topic and gets the position and orientation of the robot
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time
from std_srvs.srv import Empty
import tf

#callback function the odom pose (position+orientation) of the robot 
def odomPoseCallback(odom_msg):
    
    #get the position of the robot
    rospy.loginfo("odom pose callback")
    rospy.loginfo(rospy.get_caller_id() + " X position = %s m", '{:.2f}'.format(odom_msg.pose.pose.position.x))
    rospy.loginfo(rospy.get_caller_id() + " Y position = %s m", '{:.2f}'.format(odom_msg.pose.pose.position.y)) 
    #get the velocity of the robot    
    rospy.loginfo(rospy.get_caller_id() + " VX velocidade linear = %s m/s", '{:.2f}'.format(odom_msg.twist.twist.linear.x))
    rospy.loginfo(rospy.get_caller_id() + " VY velocidade linear = %s m/s", '{:.2f}'.format(odom_msg.twist.twist.linear.y)) 
    rospy.loginfo(rospy.get_caller_id() + " VZ velocidade linear = %s rad/s", '{:.2f}'.format(odom_msg.twist.twist.angular.z)) 

    #print the values of the orientation in quaternion   
    rospy.loginfo(rospy.get_caller_id() + " QX orientation = %s rad/s", '{:.2f}'.format(odom_msg.pose.pose.orientation.x))
    rospy.loginfo(rospy.get_caller_id() + " QY orientation = %s rad/s", '{:.2f}'.format(odom_msg.pose.pose.orientation.y)) 
    rospy.loginfo(rospy.get_caller_id() + " QZ orientation = %s rad/s", '{:.2f}'.format(odom_msg.pose.pose.orientation.z))
    rospy.loginfo(rospy.get_caller_id() + " QW orientation = %s rad/s", '{:.2f}'.format(odom_msg.pose.pose.orientation.w)) 
    
    #formulate a quaternion as a list
    quaternion = (
    odom_msg.pose.pose.orientation.x,
    odom_msg.pose.pose.orientation.y,
    odom_msg.pose.pose.orientation.z,
    odom_msg.pose.pose.orientation.w)
    
    #convert the quaternion to roll-pitch-yaw
    rpy = tf.transformations.euler_from_quaternion(quaternion)
    #extract the values of roll, pitch and yaw from the array
    roll = rpy[0]
    pitch = rpy[1]
    yaw = rpy[2]

    #print the roll, pitch and yaw
    rospy.loginfo(rospy.get_caller_id() + " The orientation of the robot along the x(roll) axis is: %s", '{:.2f}'.format(math.degrees(roll)))
    rospy.loginfo(rospy.get_caller_id() + " The orientation of the robot along the y(pitch) axis is: %s", '{:.2f}'.format(math.degrees(pitch)))    
    rospy.loginfo(rospy.get_caller_id() + " The orientation of the robot along the z(yaw) axis is: %s", '{:.2f}'.format(math.degrees(yaw)))

if __name__ == '__main__':
    try:
        #init the node
        rospy.init_node('tf_orientation_tb3', anonymous=True)
       
       #subscribe to the odom topic 
        position_topic = "/odom"
        pose_subscriber = rospy.Subscriber(position_topic, Odometry, odomPoseCallback) 
        #spin
        rospy.spin()
       
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")