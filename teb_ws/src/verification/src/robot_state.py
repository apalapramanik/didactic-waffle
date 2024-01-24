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
import matplotlib.pyplot as plt

# from verification.src.StarV.StarV.plant.dlode import DLODE


from StarV.plant.dlode import DLODE
# from verification.src.StarV.StarV.set.star import Star
# from verification.src.StarV.StarV.set.probstar import ProbStar

class robot_state:
    
    def __init__(self):
        rospy.init_node('robot_state', anonymous=True)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.states_history = []
        self.errors_history = []
        
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
        
        self.states_history.append(self.X)
        
        print("current state:", self.X, "\n")
     
        
       
        twist_cmd = Twist()
        twist_cmd.linear.x = 0.1  # Example linear velocity
        twist_cmd.angular.z = 0.1  # Example angular velocity
        self.twist_pub.publish(twist_cmd)
        
        self.U = np.array([twist_cmd.linear.x , twist_cmd.angular.z])
        
      

        # Calculate state-space matrices (A, B, C, D)
    
        self.A = np.array([[1.0, 0.0, 0.0],
                           [0.0, 1.0, 0.0],
                           [0.0, 0.0, 1.0]])

        self.B = np.array([[np.cos(theta), 0],
                           [np.sin(theta), 0],
                           [0, 1]])

        self.C = np.eye(3)  # Output all states as measurements

        self.D = np.zeros((3, 2))  # No direct dependence on control inputs

   
        
        # plant = DLODE(self.A, self.B, self.C, self.D)
        # plant.info()
        
  
        
        # self.next_state = np.dot(self.A, self.X) + np.dot(self.B, self.U)
        
        print("next state: ", self.next_state,"\n")
        
        error = self.next_state - self.X
        
        # Append current state and error to history lists
        self.states_history.append(self.X)
        self.errors_history.append(error)
        
        # Print and plot
        print("Next state:", self.next_state)
        print("Error:", error)
        
        # Plotting
        self.plot_results()
        
        print("---------------------------------------------------")
        
    def plot_results(self):
        # Plotting code
        time_steps = range(len(self.states_history))

        # Plot states
        plt.figure(1)
        plt.subplot(211)
        plt.plot(time_steps, self.states_history)
        plt.title('Current States Over Time')
        plt.xlabel('Time Steps')
        plt.ylabel('States')

        # Plot errors
        plt.subplot(212)
        plt.plot(time_steps, self.errors_history)
        plt.title('Errors Over Time')
        plt.xlabel('Time Steps')
        plt.ylabel('Errors')

        plt.tight_layout()
        plt.show()
        
        
        
        
        
if __name__ == '__main__':
   
    robot_state_calc = robot_state()
    rospy.spin()
        
        
        