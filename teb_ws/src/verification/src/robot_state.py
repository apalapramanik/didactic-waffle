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
from scipy.linalg import expm

from StarV.plant.dlode import DLODE
import numpy as np
from tb_apala.src.StarV.StarV.set.star import Star
from tb_apala.src.StarV.StarV.set.probstar import ProbStar

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
        
       
        twist_cmd = Twist()
        # twist_cmd.linear.x = 0.1  # Example linear velocity
        # twist_cmd.angular.z = 0.1  # Example angular velocity
        # self.twist_pub.publish(twist_cmd)
        
      

        # Calculate state-space matrices (A, B, C, D)
        A = np.array([[0, 0, -twist_cmd.linear.x * np.sin(theta)],
                      [0, 0, twist_cmd.linear.x * np.cos(theta)],
                      [0, 0, 0]])

        B = np.array([[np.cos(theta), 0],
                      [np.sin(theta), 0],
                      [0, 1]])

        C = np.eye(3)  # Output all states as measurements

        D = np.zeros((3, 2))  # No direct dependence on control inputs

       

        # Print the calculated matrices (for testing purposes)
        rospy.loginfo("A: \n%s", A)
        rospy.loginfo("B: \n%s", B)
        rospy.loginfo("C: \n%s", C)
        rospy.loginfo("D: \n%s", D)
      
        
        plant = DLODE(A, B, C, D)
        plant.info()
        
        
        
        
        
if __name__ == '__main__':
   
    robot_state_calc = robot_state()
    rospy.spin()
        
        
        