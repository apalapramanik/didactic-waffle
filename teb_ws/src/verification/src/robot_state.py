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
from math import cos, sin

# from verification.src.StarV.StarV.plant.dlode import DLODE




class robot_state:
    
    def __init__(self):
        rospy.init_node('robot_state', anonymous=True)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
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
        
        self.X = np.array([x, y, theta])
        
        vel = odom_msg.twist.twist.linear 
        omega = odom_msg.twist.twist.angular
        
        self.U = np.array([vel, omega])     
        
        
        if len(self.states_history) <= 19:
            self.states_history.append(self.X)
        else:
            self.states_history.pop(0)  # Remove the oldest entry
            self.states_history.append(self.X)

        print("length:", len(self.states_history))

        if len(self.states_history) == 20:
            # Convert states_history to a NumPy array for easier calculations
            states_array = np.array(self.states_history)

            # Calculate mean and standard deviation for x
            self.mean_x = np.mean(states_array[:, 0])
            self.std_x = np.std(states_array[:, 0])

            # Calculate mean and standard deviation for y
            self.mean_y = np.mean(states_array[:, 1])
            self.std_y = np.std(states_array[:, 1])

            # Calculate mean and standard deviation for theta
            self.mean_theta = np.mean(states_array[:, 2])
            self.std_theta = np.std(states_array[:, 2])


            # Print the results
            print("Mean and standard deviation for x:")
            print(f"Mean: {self.mean_x}, Standard Deviation: {self.std_x}")

            print("\nMean and standard deviation for y:")
            print(f"Mean: {self.mean_y}, Standard Deviation: {self.std_y}")

            print("\nMean and standard deviation for theta:")
            print(f"Mean: {self.mean_theta}, Standard Deviation: {self.std_theta}")
            
            self.X_initial = np.array([self.mean_x, self.mean_y, self.mean_theta])
            self.std_initial = np.array([self.std_x, self.std_y, self.std_theta])
            
            
        
        
        # with open('states.txt', 'a') as file:
        #     file.write(f"{self.X}\n")
            
        # print("current state:", self.X, "\n")
     
        
       
        # twist_cmd = Twist()
        # twist_cmd.linear.x = 0.1  # Example linear velocity
        # twist_cmd.angular.z = 0.0  # Example angular velocity
        # self.twist_pub.publish(twist_cmd)
        
        # self.U = np.array([twist_cmd.linear.x , twist_cmd.angular.z])
        
      

        # Calculate state-space matrices (A, B, C, D)
    
        self.A = np.array([[1.0, 0.0, -vel*cos(theta)],
                           [0.0, 1.0, -vel*sin(theta)],
                           [0.0, 0.0, 1.0]])

        # self.B = np.array([[np.cos(theta), 0],
        #                    [np.sin(theta), 0],
        #                    [0, 1]])

        # self.C = np.eye(3)  # Output all states as measurements

        # self.D = np.zeros((3, 2))  # No direct dependence on control inputs

   
        
        # plant = DLODE(self.A, self.B, self.C, self.D)
        # # plant.info()
        
        # s = ProbStar.rand(2)
        # print(s)
        
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
        
        
                
  
        
        # self.next_state = np.dot(self.A, self.X) + np.dot(self.B, self.U)
        
        # with open('next_states.txt', 'a') as file:
        #     file.write(f"{self.next_state}\n")
            
        
        # print("next state: ", self.next_state,"\n")
        
        # if len(self.states_history) >1:
        #     self.error = self.next_state - self.states_history[-2]
        # else:
        #     self.error = None
            
        # Append current state and error to history lists
        # self.errors_history.append(self.error)
        
        # Print and plot
        # print("Error:", self.error)
       
            
        
        # Plotting
        # self.plot_results()
        
        print("---------------------------------------------------")
        
        
if __name__ == '__main__':
   
    robot_state_calc = robot_state()
    rospy.spin()
        
        
"""
Mean and standard deviation for x:
Mean: 11.45685789395808, Standard Deviation: 5.868065846169667e-07

Mean and standard deviation for y:
Mean: -7.218021583352927, Standard Deviation: 2.981426525819482e-06

Mean and standard deviation for theta:
Mean: 1.6257967346611615, Standard Deviation: 1.031531294396443e-05

initial state: [11.45686 -7.21802  1.62578] 

"""
        
        