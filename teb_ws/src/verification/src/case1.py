#!/usr/bin/env python

"""
    Author:  Apala Pramanik 
    Project: Autonomous Robot Safety Verification in Construction site
    Advised by: Dr.Dung Hoang Tran    
    
"""

import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
# from marker_pub import marker
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
from scipy.linalg import expm
from sensor_msgs.msg import PointCloud2 as pc2
from StarV.util.plot import plot_probstar
from std_msgs.msg import Float32, String
from verification.msg import position
from math import cos, sin
import math
import csv


from StarV.plant.dlode import DLODE
from StarV.set.probstar import ProbStar


robot_width = 0.281
robot_length = 0.306
robot_height = 0.141

human_length = 1.79 #avg length (full arm) for men meters
human_width = 1.79 #avg width (full arm) for men meters
human_height = 1.740 #avg height for men meters




v_tb1 = []
w_tb1 = []
v_tb0 = []
w_tb0 = []
v_tb2 = []
w_tb2 = []

x = []
y = []


prev_time = 0.0
dt = 0.0



class marker:
 
    def publish_pose_marker(frame, name, cord_x, cord_y, cord_z, std_x, std_y, std_z, or_x, or_y, or_z, or_w):
        
        human_marker = rospy.Publisher(name, Marker, queue_size=0)
        prediction_marker_cube = Marker()
    
        
        prediction_marker_cube.header.stamp = rospy.Time.now()
        prediction_marker_cube.header.frame_id = frame #"camera_rgb_optical_frame"#"map"
        prediction_marker_cube.ns = "basic_shapes_1"
        prediction_marker_cube.id = 1
        prediction_marker_cube.type = 1
        prediction_marker_cube.pose.position.x = cord_x 
        prediction_marker_cube.pose.position.y = cord_y
        prediction_marker_cube.pose.position.z = cord_z 
        prediction_marker_cube.pose.orientation.x = or_x
        prediction_marker_cube.pose.orientation.y =  or_y
        prediction_marker_cube.pose.orientation.z = or_z
        prediction_marker_cube.pose.orientation.w = or_w
        prediction_marker_cube.scale.x = std_x
        prediction_marker_cube.scale.y = std_y
        prediction_marker_cube.scale.z = std_z
        prediction_marker_cube.color.a = 1.0
        prediction_marker_cube.color.r = 1.0
        prediction_marker_cube.color.g = 0.0
        prediction_marker_cube.color.b = 0.0
        
        #publish marker at current mean position of human:
        human_marker.publish(prediction_marker_cube)
        
    def publish_prediction_marker(frame, a, name, cord_x, cord_y, cord_z, std_x, std_y, std_z, or_x, or_y, or_z, or_w):
       
        prediction_marker = rospy.Publisher(name, Marker, queue_size=0)
        pred_marker_cube = Marker()
        
        pred_marker_cube.header.stamp = rospy.Time.now()
        pred_marker_cube.header.frame_id = frame #"map"
        pred_marker_cube.ns = "basic_shapes_1"
        pred_marker_cube.id = a
        pred_marker_cube.type = 1
        pred_marker_cube.pose.position.x = cord_x 
        pred_marker_cube.pose.position.y = cord_y
        pred_marker_cube.pose.position.z = cord_z 
        pred_marker_cube.pose.orientation.x = or_x
        pred_marker_cube.pose.orientation.y =  or_y
        pred_marker_cube.pose.orientation.z = or_z
        pred_marker_cube.pose.orientation.w = or_w
        pred_marker_cube.scale.x = std_x
        pred_marker_cube.scale.y = std_y
        pred_marker_cube.scale.z = std_z
        pred_marker_cube.color.a = 1.0
        pred_marker_cube.color.r = 0.0
        pred_marker_cube.color.g = 1.0
        pred_marker_cube.color.b = 0.0
        
        #publish marker at predicted positions of human:
        prediction_marker.publish( pred_marker_cube)

def probstar_halfspace_intersection_2d(P1, P2):
    if isinstance(P1, ProbStar) and isinstance(P2, ProbStar):
        
        l, u = P2.getRanges()
        
        H = np.array([[1,0], 
                  [-1,0],
                  [0,1],
                  [0,-1]])
        
        g = np.array([u[0], -l[0], u[1], -l[1]])
        
        g = g.reshape(4,1)
        
        
        V = P1.V
        v_new = V[0:2, 1:3] #new basis vector
        c_new =  np.array([[V[0][0]], [V[1][0]]]) # new center
        V_new = np.concatenate([c_new, v_new], axis =1) #new combined V 
        C_new = np.matmul(H, v_new) #new constarint matrix
        d_new = g-np.matmul(H,c_new) 
        d_new = d_new.reshape(4,) #new constraint vector
        new_mu = P1.mu[0:2]
        new_sig = P1.Sig[0:2,0:2]
        new_pred_lb = P1.pred_lb[0:2]
        
        new_pred_ub = P1.pred_ub[0:2]
        
        intersection = ProbStar(V_new,C_new,d_new,new_mu, new_sig,new_pred_lb,new_pred_ub)
        # plot_probstar(intersection)
        collision_probability = intersection.estimateProbability()
        # if collision_probability>0:
            # print(intersection.__str__())
        
        return collision_probability
    
    else:
        return("Error: Input is not a probstar")
    

class robot_human_state:
    
    def __init__(self):
        
        rospy.init_node('tb3_0_collision_node', anonymous=True)
    
        
        self.odom_sub0 = rospy.Subscriber('tb3_0/odom', Odometry, self.odom_callback_tb3_0,queue_size=100)
        self.odom_sub1 = rospy.Subscriber('tb3_1/odom', Odometry, self.odom_callback_tb3_1,queue_size=100)

         
        self.prev_time = 0.0
        self.dt = 0.0
        
        self.H = np.array([[1, 0, 0], [0, 1, 0]]) #2x3
        self.Q = np.diag([0.01, 0.01, 0.01]) #3x3
        self.R = np.diag([0.01, 0.01]) #2x2
        
        self.x = np.array([[0.0],[0.0],[0.0]]) #3x1
        self.z = np.array([[0.0],[0.0]])
        self.u = 0
        self.P   = np.array([[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]])
        self.P_k = np.array([[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]])
        self.v_x = 0
        self.v_y = 0
        
        self.probstars_human = []
        self.probstars_tb3_0 = []
        self.probstars_tb3_1 = []
        self.probstars_tb3_2 = []
        self.flag = "no"
        
        self.collision_prob_tb3_01 = rospy.Publisher("tb3_01/collision_prob", Float32,queue_size=100 )
        self.collision_prob_tb3_02 = rospy.Publisher("tb3_02/collision_prob", Float32,queue_size=100 )
        
    
    def odom_callback_tb3_0(self, odom_msg):
       
        # Extract pose and twist information from odometry message
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        quaternion = (
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z,
            odom_msg.pose.pose.orientation.w
        )
        
        current_vel_tb0 = odom_msg.twist.twist.linear.x
        current_omega_tb0 = odom_msg.twist.twist.angular.z 
    
        
        _, _, theta = euler_from_quaternion(quaternion)  
        # theta = 0.5             
    
        self.X_tb0 = np.array([x, y, theta])  
        print("odom tb3_0: ", self.X_tb0 )
  
        self.U_tb0 = np.array([current_vel_tb0, current_omega_tb0])   
        
        self.c_tb0 = (np.expand_dims(self.X_tb0, axis=0)).transpose()
        self.dev_tb0 = np.array([0.281, 0.306, 0.001])
        self.v_tb0 = np.diag(self.dev_tb0)
        
        self.V_tb0 = np.concatenate([self.c_tb0, self.v_tb0], axis =1)
        self.C_tb0 = []
        self.d_tb0 = []
        self.mu_tb0 = np.zeros(3)        
        self.sigma_tb0 = np.diag(np.ones(3))         
        self.pred_lb_tb0 = np.ones(3) * 4.5 * -1
        self.pred_ub_tb0 = np.ones(3) * 4.5 
        
        self.A_tb0= np.array([[1.0, 0.0, 0.0],
                           [0.0, 1.0, 0.0],
                           [0.0, 0.0, 1.0]])
        
        self.dtm_tb0 = 0.7 #odom time period = 0.03 / no of obs
        
        self.b_tb0 = np.array([[cos(theta)*self.dtm_tb0, 0.0],
                              [sin(theta)*self.dtm_tb0, 0.0],
                              [0.0, 1.0]])
    
        init_probstar_tb0 = ProbStar(self.V_tb0, self.C_tb0, self.d_tb0, self.mu_tb0,
                                     self.sigma_tb0, self.pred_lb_tb0, self.pred_ub_tb0)     
        
        
        self.bu_tb0 = np.matmul(self.b_tb0, self.U_tb0).flatten()
        
        next_prob_star_tb0 = init_probstar_tb0.affineMap(self.A_tb0, self.bu_tb0)
     
        for i in range(3):
            next_prob_star_tb0  = next_prob_star_tb0.affineMap(self.A_tb0, self.bu_tb0)
            self.probstars_tb3_0.append(next_prob_star_tb0)
            new_x =  next_prob_star_tb0.V[0][0]
            new_y = next_prob_star_tb0.V[1][0]
            new_theta = next_prob_star_tb0.V[2][0]
            new_quaternion = quaternion_from_euler(0,0,new_theta)
          
            marker.publish_prediction_marker("map", i, name = "pred_robot_tb3_0", cord_x= new_x, cord_y=new_y, 
                                                        cord_z= 0.0, std_x=robot_length,
                                                        std_y = robot_width, std_z = robot_height,
                                                        or_x = new_quaternion[0],or_y = new_quaternion[1],
                                                        or_z=new_quaternion[2],or_w=new_quaternion[3])    
        prob_msg1 = Float32()        
        p_set1 = []
        for i in range(7) :
           
            p1 = probstar_halfspace_intersection_2d(self.probstars_tb3_0[i],  self.init_probstar_tb1)            
            print("tb3_0", self.probstars_tb3_0[i].V)
            
            p_set1.append(p1)
            # print("prob1: ", p1)
               
        p_max1 =  max(p_set1) 
        prob_msg1.data = p_max1
        self.collision_prob_tb3_01.publish(prob_msg1)
        # print("max:",p_max1)
        print("...........................................................................")
        
      
                
                
        
    def odom_callback_tb3_1(self, odom_msg):
       
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
                  
    
        self.X_tb1 = np.array([x, y, theta])  
        print("odom tb3_1: ", self.X_tb1 )
     
        current_vel_tb1 = odom_msg.twist.twist.linear.x
        current_omega_tb1 = odom_msg.twist.twist.angular.z 
        
        self.U_tb1 = np.array([current_vel_tb1, current_omega_tb1]) 
        
        self.c_tb1 = (np.expand_dims(self.X_tb1, axis=0)).transpose()
        self.dev_tb1 = np.array([0.281, 0.306, 0.001])
        self.v_tb1 = np.diag(self.dev_tb1)
        
        self.V_tb1 = np.concatenate([self.c_tb1 ,self.v_tb1], axis =1)
        self.C_tb1 = []
        self.d_tb1 = []
        self.mu_tb1 = np.zeros(3)        
        self.sigma_tb1 = np.diag(np.ones(3))         
        self.pred_lb_tb1 = np.ones(3) * 4.5 * -1
        self.pred_ub_tb1 = np.ones(3) * 4.5 
        
        
        
        self.A_tb1= np.array([[1.0, 0.0, 0.0],
                           [0.0, 1.0, 0.0],
                           [0.0, 0.0, 1.0]])
        
        self.dtm_tb1 = 0.9 #odom time period = 0.03 / no of obs #0.7, 0.9
        
        self.b_tb1 = np.array([[cos(theta)*self.dtm_tb1, 0.0],
                              [sin(theta)*self.dtm_tb1, 0.0],
                              [0.0, 1.0]])
    
        self.init_probstar_tb1 = ProbStar(self.V_tb1, self.C_tb1, self.d_tb1, self.mu_tb1,
                                     self.sigma_tb1, self.pred_lb_tb1, self.pred_ub_tb1)
        
        print("tb3_1", self.init_probstar_tb1.V)
        self.bu_tb1 = np.matmul(self.b_tb1, self.U_tb1).flatten()
        
        # self.next_prob_star_tb1 = self.init_probstar_tb1.affineMap(self.A_tb1, self.bu_tb1)
     
        # for i in range(5):
        #     self.next_prob_star_tb1  = self.next_prob_star_tb1.affineMap(self.A_tb1, self.bu_tb1)
        #     self.probstars_tb3_1.append(self.next_prob_star_tb1)
        #     new_x =  self.next_prob_star_tb1.V[0][0]
        #     new_y = self.next_prob_star_tb1.V[1][0]
        #     new_theta = self.next_prob_star_tb1.V[2][0]
        #     new_quaternion = quaternion_from_euler(0,0,new_theta)
        #     marker.publish_prediction_marker("map", i, name = "pred_robot_tb3_1", cord_x= new_x, cord_y=new_y, 
        #                                                 cord_z= 0.0, std_x=robot_length,
        #                                                 std_y = robot_width, std_z = robot_height,
        #                                                 or_x = new_quaternion[0],or_y = new_quaternion[1],
        #                                                 or_z=new_quaternion[2],or_w=new_quaternion[3])   
            

       
if __name__ == '__main__':
   
    robot_state_calc = robot_human_state()
    
    rospy.spin()
    
    
  
      

"""

Robot:

Mean and standard deviation for x:
Mean: 11.45685789395808, Standard Deviation: 5.868065846169667e-07

Mean and standard deviation for y:
Mean: -7.218021583352927, Standard Deviation: 2.981426525819482e-06

Mean and standard deviation for theta:
Mean: 1.6257967346611615, Standard Deviation: 1.031531294396443e-05

initial state: [11.45686 -7.21802  1.62578] 


"""
 
        