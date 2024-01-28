#!/usr/bin/env python

"""
    Author:  Apala Pramanik 
    Project: Autonomous Robot Safety Verification in Construction site
    Advised by: Dr.Dung Hoang Tran    
    
"""

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np
from scipy.linalg import expm
from sensor_msgs.msg import PointCloud2 as pc2
from math import cos, sin
from reachability_node import *
import math

robot_width = 0.281
robot_length = 0.306
std_dev=2 
steps=3





class robot_human_state:
    
    def __init__(self):
        rospy.init_node('robot_state', anonymous=True)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback,queue_size=10) 
        self.pc_human_sub = rospy.Subscriber("projected",pc2,self.human_pc_callback,queue_size=10)
        
        self.states_history = []
        self.errors_history = []
        
        # Initialize state variables
        self.X = np.array([0.0, 0.0, 0.0])  # [x, y, theta]
        self.last_time = rospy.Time.now()

     

    def human_pc_callback(self, pose_msg):
        
        pcl_np = pointcloud2_to_numpy(pose_msg)
        mean_value = np.mean(pcl_np, axis=0)
        std_deviation = np.std(pcl_np, axis=0)
        
        self.x_mean = mean_value[0]
        self.y_mean = mean_value[1]
        
        self.x_std = std_deviation[0]
        self.y_std = std_deviation[1]
        
        # Define a bounding box based on mean and standard deviation
        bounding_box_size_multiplier = 2
        bounding_box_width = bounding_box_size_multiplier * self.x_std
        bounding_box_height = bounding_box_size_multiplier * self.y_std

        # Calculate bounding box vertices
        x_min = self.x_mean - bounding_box_width / 2
        x_max = self.x_mean + bounding_box_width / 2
        y_min = self.y_mean - bounding_box_height / 2
        y_max = self.y_mean + bounding_box_height / 2

        # Bounding box vertices in the order: top-left, top-right, bottom-right, bottom-left
        self.bb_vertices_human = [(x_min, y_max), (x_max, y_max), (x_max, y_min), (x_min, y_min)]
        

   
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
        
        vel_x = odom_msg.twist.twist.linear.x
        vel_y = odom_msg.twist.twist.linear.y
        vel_z = odom_msg.twist.twist.linear.z
        
        vel = math.sqrt(vel_x**2 + vel_y**2 + vel_z**2)
        
        omega = odom_msg.twist.twist.angular
        
        self.U = np.array([vel, omega])    
        
        dt = 0.025 #model_dt = 0.25/10
        
        
    
        self.A = np.array([[1.0, 0.0, -1 * vel*cos(theta)*dt],
                           [0.0, 1.0, -1 * vel*sin(theta)*dt],
                           [0.0, 0.0, 1.0]])
        
        self.X_initial = self.X
        self.std_initial = np.array([0.0, 0.0, 0.0])
        self.U_initial = self.U
        
        probstars = compute_reachability(bb_vertices_human =  self.bb_vertices_human ,  X_initial = self.X_initial, std_initial = self.std_initial , A = self.A)
        
        for i in range(steps):
            print("Probstar ", i, ": ", probstars[i])          
 
        
                
        print("---------------------------------------------------")
 
def pointcloud2_to_numpy(pointcloud_msg):
    # Convert the PointCloud2 message to a NumPy array
    numpy_array = np.frombuffer(pointcloud_msg.data, dtype=np.float32).reshape(-1, pointcloud_msg.point_step // 4)

    # Extract x, y, and z coordinates
    x = numpy_array[:, 0]
    y = numpy_array[:, 1]
    z = numpy_array[:, 2]

    # Create a NumPy array with x, y, z coordinates
    points = np.column_stack((x, y, z))

    return points             
        
if __name__ == '__main__':
   
    robot_state_calc = robot_human_state()
    rospy.spin()
  
        
        
"""

 # if len(self.states_history) <= 19:
        #     self.states_history.append(self.X)
        # else:
        #     self.states_history.pop(0)  # Remove the oldest entry
        #     self.states_history.append(self.X)

        # print("length:", len(self.states_history))

        # if len(self.states_history) == 20:
        #     # Convert states_history to a NumPy array for easier calculations
        #     states_array = np.array(self.states_history)

        #     # Calculate mean and standard deviation for x
        #     self.mean_x = np.mean(states_array[:, 0])
        #     self.std_x = np.std(states_array[:, 0])

        #     # Calculate mean and standard deviation for y
        #     self.mean_y = np.mean(states_array[:, 1])
        #     self.std_y = np.std(states_array[:, 1])

        #     # Calculate mean and standard deviation for theta
        #     self.mean_theta = np.mean(states_array[:, 2])
        #     self.std_theta = np.std(states_array[:, 2])


        #     # Print the results
        #     print("Mean and standard deviation for x:")
        #     print(f"Mean: {self.mean_x}, Standard Deviation: {self.std_x}")

        #     print("\nMean and standard deviation for y:")
        #     print(f"Mean: {self.mean_y}, Standard Deviation: {self.std_y}")

        #     print("\nMean and standard deviation for theta:")
        #     print(f"Mean: {self.mean_theta}, Standard Deviation: {self.std_theta}")
            
        #     self.X_initial = np.array([self.mean_x, self.mean_y, self.mean_theta])
        #     self.std_initial = np.array([self.std_x, self.std_y, self.std_theta])
Robot:

Mean and standard deviation for x:
Mean: 11.45685789395808, Standard Deviation: 5.868065846169667e-07

Mean and standard deviation for y:
Mean: -7.218021583352927, Standard Deviation: 2.981426525819482e-06

Mean and standard deviation for theta:
Mean: 1.6257967346611615, Standard Deviation: 1.031531294396443e-05

initial state: [11.45686 -7.21802  1.62578] 

"""

        