#!/usr/bin/env python

"""
    Author:  Apala Pramanik 
    Project: Autonomous Robot Safety Verification in Construction site
    Advised by: Dr.Dung Hoang Tran    
    
"""

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import numpy as np

class robot_state:
    
    def __init__(self):
        rospy.init_node('robot_state', anonymous=True)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Initialize state variables
        self.X = np.array([0.0, 0.0, 0.0])  # [x, y, theta]
        self.last_time = rospy.Time.now()
        
        
    def odom_callback(self, odom_msg):
        
        # Extract pose and twist information from odometry message
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        quaternion = (
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z,
            odom_msg.pose.pose.orientation.w
        )
        _, _, theta = euler_from_quaternion(quaternion)
        
        # Update state variables
        self.X = np.array([x, y, theta])
        
        
        