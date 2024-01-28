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
from probstar import ProbStar

# from StarV.plant.dlode import DLODE
# from StarV.set.probstar import ProbStar

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
        self.bb_vertices_human = np.array([0.0, 0.0, 0.0, 0.0])

     

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
        
        self.A_final = np.zeros((8,8))
        
        self.A_final[:3 , :3] = self.A
        

        # print(self.A_final)
        
        # self.X_initial = self.X
        # self.std_initial = np.array([0.0, 0.0, 0.0])
        # self.U_initial = self.U
        
        self.X_initial = np.array([x, y, theta, 0.0,0.0,0.0,0.0,0.0])
        self.std_initial = np.array([0.0, 0.0, 0.0, 0.0,0.0,0.0,0.0,0.0])
        self.U_initial =np.array([vel, omega,0.0, 0.0 ])
        
        probstars = compute_reachability(bb_vertices_human =  self.bb_vertices_human ,  
                                         X_initial = self.X_initial, std_initial = self.std_initial ,
                                         A = self.A_final, angle = theta)
        
        probabilities = []
        for probstar in probstars:
            probabilities.append(probstar.estimateProbability())
        
       
        print(probabilities)
        
       
        
        # for i in range(steps):
        #     print("Probstar ", i, ": ", probstars[i])          
 
        
                
        print("---------------------------------------------------")
        
def estimate_probstar_probability(probstar):
    prob = probstar.estimateProbability()
    return prob

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

def compute_reachability( bb_vertices_human,  X_initial, std_initial,  A , angle):  
    


    # Convert Initial State to Probstar 
    c = (np.expand_dims(X_initial, axis=0)).transpose()
    V = np.diag(std_initial)
    # print("v:", V.shape)
    n_sigma = np.diag(np.ones(X_initial.shape[0]))
    n_mu = np.zeros(X_initial.shape[0])
    # print(len(n_mu.shape))
    l = np.ones(X_initial.shape[0]) * std_dev * -1
    u = np.ones(X_initial.shape[0]) * std_dev
    
    probstars = []

    # Iterate Model Until k = reachability_start_idx
    for i in range(steps):
        for j in range(10):
            # print("A:", A.shape)
            c = np.matmul(A, c)
            V = np.matmul(A, V)
            # print("v:", V.shape)

    
    for i in range(steps):
        for j in range(10):
            c = np.matmul(A,c)
            V = np.matmul(A, V)

        # Apply Collision Bounding Predicate
      
        H,g = predicates(angle, bb_vertices_human, robot_width, robot_length)
        # print("H:", H.shape, "g:", g.shape, "v:", V.shape)
        C = np.matmul(H,V)
        d = g-np.matmul(H,c)
        d = np.asarray(d).squeeze()
        c_V_Combine =  np.concatenate([c,V], axis=1)
        c_V_Combine = np.asarray(c_V_Combine)
        V = np.asarray(V)
        C = np.asarray(C)
        probstar = ProbStar(c_V_Combine,C,d,n_mu,n_sigma)
        probstars.append(probstar)
    return probstars





def bb_vertices(width, length, angle):
   
    x1 = (length/2) * math.cos(angle) - (width/2) * math.sin(angle)
    y1 = (width/2) * math.cos(angle) + (length/2) * math.sin(angle)
    x2 = -(length/2) * math.cos(angle) - (width/2) * math.sin(angle)
    y2 = (width/2) * math.cos(angle) - (length/2) * math.sin(angle)
    x3 = (length/2) * math.cos(angle) + (width/2) * math.sin(angle)
    y3 = -(width/2) * math.cos(angle) + (length/2) * math.sin(angle)
    x4 = -(length/2) * math.cos(angle) + (width/2) * math.sin(angle)
    y4 = -(width/2) * math.cos(angle) - (length/2) * math.sin(angle)
    bounding_box_vertices = [(x1, y1), (x2, y2), (x3, y3), (x4, y4)]
    return bounding_box_vertices

def add_edges(edges, bb_vertices, box_index):
        for i in range(len(bb_vertices)):
            next_idx = (i + 1) % len(bb_vertices)
            dx = bb_vertices[next_idx][0] - bb_vertices[i][0]
            dy = bb_vertices[next_idx][1] - bb_vertices[i][1]
            edges.append([dx, dy, box_index, i])

def minkowski_difference_2d_convex_hull(bb1, bb2):
    
    # Convert to minkowski addition problem by negating vertices of the second bounding box
    bb2_negated = [[-x, -y] for x, y in bb2]

    # Sort vertices by polar angle
    bb1.sort(key=lambda vertex: math.atan2(vertex[1], vertex[0]))
    bb2_negated.sort(key=lambda vertex: math.atan2(vertex[1], vertex[0]))

    # Create edges for both bounding boxes
    edges = []        

    add_edges(edges, bb1, 1)
    add_edges(edges, bb2_negated, 2)

    # Sort edges by polar angle
    edges.sort(key=lambda edge: math.atan2(edge[1], edge[0]))

    # Calculate the starting point as the sum of positions of starting vertices in the first two edges
    first_edge_bb1 = next(edge for edge in edges if edge[2] == 1)
    first_edge_bb2 = next(edge for edge in edges if edge[2] == 2)
    starting_x = bb1[first_edge_bb1[3]][0] + bb2[first_edge_bb2[3]][0]
    starting_y = bb1[first_edge_bb1[3]][1] + bb2[first_edge_bb2[3]][1]

    minkowski_sum_bbox = []
    current_point = (starting_x, starting_y)

    # Calculate Minkowski sum
    for edge in edges:
        current_point = (current_point[0] + edge[0], current_point[1] + edge[1])
        minkowski_sum_bbox.append(current_point)

    return minkowski_sum_bbox

def convex_hull_vertex_array_to_linear_constraint(convex_hull_array):
    num_vertices = len(convex_hull_array)
    C = np.zeros((num_vertices, 2))
    d = np.zeros((num_vertices, 1))

    for idx in range(num_vertices):
        x1, y1 = convex_hull_array[idx]
        x2, y2 = convex_hull_array[(idx + 1) % num_vertices]

        C[idx, 0] = -(y2 - y1)
        C[idx, 1] = x2 - x1
        d[idx, 0] = -(y1 * (x2 - x1) - x1 * (y2 - y1))
    # print("C,d shape: " , C.shape, d.shape)

    return C, d

def predicates( angle, bb_vertices_human, robot_width, robot_length):  
    bb_vertices_robot = bb_vertices(robot_width, robot_length,  angle)    
    minkowski_difference = minkowski_difference_2d_convex_hull(bb_vertices_robot,bb_vertices_human)
    # print(minkowski_difference)
    C_s,d_s = convex_hull_vertex_array_to_linear_constraint(minkowski_difference)
    # print(C_s.shape, C_s)
    C = np.hstack([C_s, np.zeros((C_s.shape[0],2)),-1*C_s,np.zeros((C_s.shape[0],2))])
    d=d_s   
    # print(C.shape, C)
    
    return C,d
        
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

        