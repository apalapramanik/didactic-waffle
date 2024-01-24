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
from std_msgs.msg import Float32

# from verification.src.StarV.StarV.plant.dlode import DLODE


from StarV.plant.dlode import DLODE
from StarV.set.star import Star
from StarV.set.probstar import ProbStar

class robot_state:
    
    def __init__(self):
        rospy.init_node('robot_state', anonymous=True)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.error_pub = rospy.Publisher('/state_error', Float32, queue_size=10)  
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
        
        # with open('states.txt', 'a') as file:
        #     file.write(f"{self.X}\n")
            
        print("current state:", self.X, "\n")
     
        
       
        twist_cmd = Twist()
        twist_cmd.linear.x = 0.1  # Example linear velocity
        twist_cmd.angular.z = 0.0  # Example angular velocity
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

   
        
        plant = DLODE(self.A, self.B, self.C, self.D)
        # plant.info()
        
        s = ProbStar.rand(2)
        print(s)
        
        """
        
        X0 = ProbStar.rand(2)
        X1, Y1 = plant.stepReach(X0)
        U0 = np.array([1.0, 0.5])
        X2, Y2 = plant.stepReach(X0, U0)
        
        # Define Distribution for Initial State Space Vector
        X_0,sigma_0,U_0 = initial_state.initial_state(pose_history,actuation_history,pose_dt_history)
        V_pi,zeta_pi,V_omega,zeta_omega = U_0

        # Convert Initial State to Probstar 
        c = (np.expand_dims(X_0, axis=0)).transpose()
        V = np.diag(sigma_0)
        n_sigma = np.diag(np.ones(X_0.shape[0]))
        n_mu = np.zeros(X_0.shape[0])
        l = np.ones(X_0.shape[0]) * standard_deviations * -1
        u = np.ones(X_0.shape[0]) * standard_deviations
        probstars = []
        
        """
        
        
                
  
        
        self.next_state = np.dot(self.A, self.X) + np.dot(self.B, self.U)
        
        # with open('next_states.txt', 'a') as file:
        #     file.write(f"{self.next_state}\n")
            
        
        print("next state: ", self.next_state,"\n")
        
        if len(self.states_history) >1:
            self.error = self.next_state - self.states_history[-2]
        else:
            self.error = None
            
        # Append current state and error to history lists
        self.errors_history.append(self.error)
        
        # Print and plot
        print("Error:", self.error)
       
            
        
        # Plotting
        # self.plot_results()
        
        print("---------------------------------------------------")
        
        
if __name__ == '__main__':
   
    robot_state_calc = robot_state()
    rospy.spin()
        
        
        