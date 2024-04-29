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
from math import cos, sin
import math
# from probstar import ProbStar

from StarV.plant.dlode import DLODE
from StarV.set.probstar import ProbStar


robot_width = 0.281
robot_length = 0.306
std_dev=2 
steps=3
pose_history=[]
human_length = 1.79 #avg length (full arm) for men
human_width = 1.79 #avg width (full arm) for men
human_height = 1.740 #avg height for men

robot_width = 0.281
robot_length = 0.306
robot_height = 0.141

v_tb1 = []
w_tb1 = []
v_tb0 = []
w_tb0 = []
v_tb2 = []
w_tb2 = []

x = []
y = []

class kalmanFilter:
    
    def __init__(self):
        
        self.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]]) #2x4
        self.Q = np.diag([1.0, 1.0, 1.0, 1.0]) #4x4
        self.R = np.diag([1.0, 1.0]) #2x2
        
        # self.x = np.array([[0.0],[0.0],[0.0]]) #3x1
        self.z = np.array([[0.0],[0.0]])
        self.P   = np.array([[0.0,0.0,0.0, 0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0]])
        self.P_k = np.array([[0.0,0.0,0.0, 0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0]])
        
        
    def predict_update(self, probstar, dt):
        
        if isinstance(probstar, ProbStar):
            
            # Process model
            self.A = np.array([[1, 0, dt, 0],
                                [0, 1, 0, dt],
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]])
            
            self.x_ps = probstar
            
            
            #update error covariance p_k
            self.x_k_ps = self.x_ps.affineMap(self.A)
            # print(self.x_k_ps)
            T = np.matmul(self.P, self.A.transpose())
            self.p_k = np.matmul(self.A, T) + self.Q
            
            # Compute Kalman gain : K
            T = np.matmul(self.P_k, self.H.transpose())
            T = np.linalg.inv(np.matmul(self.H, T) + self.R)
            T = np.matmul(self.H.transpose(), T)
            self.K = np.matmul(self.P_k, T)
            I = np.eye(4)
            self.M = I - np.matmul(self.K, self.H)
            self.N = np.matmul(self.K, self.z).flatten()
            
            #prediction        
            self.x_ps = self.x_k_ps.affineMap(self.M, self.N)
            # print(self.x_k_ps)
            self.P = np.matmul((np.eye(self.H.shape[1]) - np.matmul(self.K, self.H)), self.P_k)
            
            # print("Estimated pose:", self.x_ps.C[0], self.x_ps.C[1])
        return self.x_ps

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


class robot_human_state:
    
    def __init__(self):
        
        rospy.init_node('reachability_analysis', anonymous=True)
        
        self.odom_sub1 = rospy.Subscriber('tb3_0/odom', Odometry, self.odom_callback_tb3_0,queue_size=10) 
        self.odom_sub2 = rospy.Subscriber('tb3_1/odom', Odometry, self.odom_callback_tb3_1,queue_size=10)
        self.odom_sub3 = rospy.Subscriber('tb3_2/odom', Odometry, self.odom_callback_tb3_2,queue_size=10)
        self.states_history = []
        self.errors_history = []
        self.probstars =[]      
        
        
        
        
        self.pc_human_sub = rospy.Subscriber("projected",pc2,self.human_pc_callback,queue_size=10)
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
        
      
         
        
    def human_pc_callback(self, pose_msg):       
        
        
        self.current_time = rospy.Time.now().to_sec()
        self.dt = (self.current_time - self.prev_time)
        self.prev_time = self.current_time

        pcl_np = pointcloud2_to_numpy(pose_msg)
        self.pose_x = np.mean(pcl_np[:, 0])
        self.pose_y = np.mean(pcl_np[:, 2])

        x.append(self.pose_x)
        y.append(self.pose_y)

        # Calculate velocities if there are enough data points
        if len(x) > 1 and len(y) > 1:
            self.v_x = (x[-1] - x[-2]) / self.dt
            self.v_y = (y[-1] - y[-2]) / self.dt
        else:
            self.v_x = 0.0
            self.v_y = 0.0
            
        # print(self.v_x, self.v_y)
        self.dt = round(self.dt, 2)
        print("dt:", self.dt)
        print("vx:", self.v_x)
        print("vy:", self.v_y)

        self.z = np.array([[self.pose_x], [self.pose_y]])
        self.x_human = np.array([[self.z[0, 0]], [self.z[1, 0]], [self.v_x], [self.v_y]])
        print(self.x_human)
        
        #initial probstar  
        self.c_human = self.x_human
        self.dev_human = np.array([human_length, human_width, 0.001, 0.001])    
        self.v_human = np.diag(self.deviation)
        self.V_human = np.concatenate([self.c_human, self.v_human], axis =1)
        self.mu_human = np.zeros(4)
        self.sigma_human = np.diag(np.ones(4))        
        self.pred_lb_human = np.ones(4) * 4.5 * -1
        self.pred_ub_human = np.ones(4) * 4.5 
        
        self.C_human = []
        self.d_human = []
        
        init_probstar_human = ProbStar(self.V_human, self.C_human, self.d_human,self.mu_human, 
                                       self.sigma_human, self.pred_lb_human,self.pred_ub_human)
       
        
        
        kf = kalmanFilter()        
        next_probstar_human = kf.predict_update(init_probstar_human, self.dt)    
     
        
        for i in range(5):
            next_probstar_human = kf.predict_update(next_probstar_human, self.dt)
          
            new_x =  next_probstar_human.V[0][0] + (self.z[0]*next_probstar_human.V[0][1]) + (self.v_x * next_probstar_human.V[0][3])
            new_y = next_probstar_human.V[1][0] + (self.z[1]*next_probstar_human.V[1][2]) + (self.v_y * next_probstar_human.V[1][4])
            print("pose ", i ,": ",new_x[0], new_y[0])
           
            marker.publish_prediction_marker("camera_rgb_optical_frame",i, name = "pred_human", cord_x= new_x[0], cord_y=new_y[0], 
                                                        cord_z= 0.0, std_x=0.5,
                                                        std_y = 0.5, std_z = 0.5,
                                                        or_x = 1.0,or_y =1.0,
                                                        or_z=0.0,or_w=0.0)  
            
      
         
   
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
        
        
        
        # if(len(v_tb0))==0:
        #     v_tb0.append(current_vel_tb0)
        #     w_tb0.append(current_omega_tb0)
        #     input_vel_tb0 = current_vel_tb0
        #     input_omega_tb0 = current_omega_tb0
            
        # elif(len(v_tb0)==1):
        #     v_tb0.append(current_vel_tb0)
        #     w_tb0.append(current_omega_tb0)
        #     input_vel_tb0 = v_tb0[1] - v_tb0[0]
        #     input_omega_tb0 = w_tb0[1] - w_tb0[0]
            
        # else:
        #     v_tb0.pop(0)
        #     w_tb0.pop(0)
        #     v_tb0.append(current_vel_tb0)
        #     w_tb0.append(current_omega_tb0)
        #     input_vel_tb0 = v_tb0[1] - v_tb0[0]
        #     input_omega_tb0 = w_tb0[1] - w_tb0[0]
            
            
        
        _, _, theta = euler_from_quaternion(quaternion)  
        # theta = 0.5             
    
        self.X_tb0 = np.array([x, y, theta])  
     
        
        # self.U_tb0 = np.array([input_vel_tb0, input_omega_tb0])
        # print("u: ", self.U) 
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
        
        self.dtm_tb0 = 0.9 #odom time period = 0.03 / no of obs
        
        self.b_tb0 = np.array([[cos(theta)*self.dtm_tb0, 0.0],
                              [sin(theta)*self.dtm_tb0, 0.0],
                              [0.0, 1.0]])
        
       
    
        init_probstar_tb0 = ProbStar(self.V_tb0, self.C_tb0, self.d_tb0, self.mu_tb0,
                                     self.sigma_tb0, self.pred_lb_tb0, self.pred_ub_tb0)
      
        
        
        self.bu_tb0 = np.matmul(self.b_tb0, self.U_tb0).flatten()
        
        
        
        
        next_prob_star_tb0 = init_probstar_tb0.affineMap(self.A_tb0, self.bu_tb0)
     
        for i in range(5):
            next_prob_star_tb0  = next_prob_star_tb0.affineMap(self.A_tb0, self.bu_tb0)
            new_x =  next_prob_star_tb0.V[0][0]
            new_y = next_prob_star_tb0.V[1][0]
            new_theta = next_prob_star_tb0.V[2][0]
            new_quaternion = quaternion_from_euler(0,0,new_theta)
            # print("pose ", i ,": ",new_x, new_y)     
            marker.publish_prediction_marker("map", i, name = "pred_robot_tb3_0", cord_x= new_x, cord_y=new_y, 
                                                        cord_z= 0.0, std_x=robot_length,
                                                        std_y = robot_width, std_z = robot_height,
                                                        or_x = new_quaternion[0],or_y = new_quaternion[1],
                                                        or_z=new_quaternion[2],or_w=new_quaternion[3])      
        
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
        # theta = 0.5             
    
        self.X_tb1 = np.array([x, y, theta])  
     
        current_vel_tb1 = odom_msg.twist.twist.linear.x
        current_omega_tb1 = odom_msg.twist.twist.angular.z 
        
        
        
        # if(len(v_tb1))==0:
        #     v_tb1.append(current_vel_tb1)
        #     w_tb1.append(current_omega_tb1)
           
        #     input_vel_tb1 = current_vel_tb1
        #     input_omega_tb1 = current_omega_tb1
            
        # elif(len(v_tb1)==1):
        #     v_tb1.append(current_vel_tb1)
        #     w_tb1.append(current_omega_tb1)
            
        #     input_vel_tb1 = v_tb1[1] - v_tb1[0]
        #     input_omega_tb1 = w_tb1[1] - w_tb1[0]
            
        # else:
        #     v_tb1.pop(0)
        #     w_tb1.pop(0)
        #     v_tb1.append(current_vel_tb1)
        #     w_tb1.append(current_omega_tb1)
            
        #     input_vel_tb1 = v_tb1[1] - v_tb1[0]
        #     input_omega_tb1 = w_tb1[1] - w_tb1[0]  
            
        # self.U_tb1 = np.array([input_vel_tb1, input_omega_tb1]) 
        
        self.U_tb1 = np.array([current_vel_tb1, current_omega_tb1]) 
        
        
        # print("input:", self.U_tb1)
        
        
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
        
        self.dtm_tb1 = 0.7 #odom time period = 0.03 / no of obs #0.7, 0.9
        
        self.b_tb1 = np.array([[cos(theta)*self.dtm_tb1, 0.0],
                              [sin(theta)*self.dtm_tb1, 0.0],
                              [0.0, 1.0]])
        
       
    
        init_probstar_tb1 = ProbStar(self.V_tb1, self.C_tb1, self.d_tb1, self.mu_tb1,
                                     self.sigma_tb1, self.pred_lb_tb1, self.pred_ub_tb1)
      
        
        
        self.bu_tb1 = np.matmul(self.b_tb1, self.U_tb1).flatten()
        
        
        
        
        next_prob_star_tb1 = init_probstar_tb1.affineMap(self.A_tb1, self.bu_tb1)
     
        for i in range(5):
            next_prob_star_tb1  = next_prob_star_tb1.affineMap(self.A_tb1, self.bu_tb1)
            new_x =  next_prob_star_tb1.V[0][0]
            new_y = next_prob_star_tb1.V[1][0]
            new_theta = next_prob_star_tb1.V[2][0]
            new_quaternion = quaternion_from_euler(0,0,new_theta)
            # print(new_x, new_y)
            
               
            marker.publish_prediction_marker("map", i, name = "pred_robot_tb3_1", cord_x= new_x, cord_y=new_y, 
                                                        cord_z= 0.0, std_x=robot_length,
                                                        std_y = robot_width, std_z = robot_height,
                                                        or_x = new_quaternion[0],or_y = new_quaternion[1],
                                                        or_z=new_quaternion[2],or_w=new_quaternion[3])     
          
        print("------------------------------------")        
  
    def odom_callback_tb3_2(self, odom_msg):
        
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
        # theta = 0.5             

        self.X_tb2 = np.array([x, y, theta])  
        
        current_vel_tb2 = odom_msg.twist.twist.linear.x
        current_omega_tb2 = odom_msg.twist.twist.angular.z 
        
        
        
        # if(len(v_tb2))==0:
        #     v_tb2.append(current_vel_tb2)
        #     w_tb2.append(current_omega_tb2)
        #     input_vel_tb2 = current_vel_tb2
        #     input_omega_tb2 = current_omega_tb2
            
        # elif(len(v_tb2)==1):
        #     v_tb2.append(current_vel_tb2)
        #     w_tb2.append(current_omega_tb2)
        #     input_vel_tb2 = v_tb2[1] - v_tb2[0]
        #     input_omega_tb2 = w_tb2[1] - w_tb2[0]
            
        # else:
        #     v_tb2.pop(0)
        #     w_tb2.pop(0)
        #     v_tb2.append(current_vel_tb2)
        #     w_tb2.append(current_omega_tb2)
        #     input_vel_tb2 = v_tb2[-1] - v_tb2[-2]
        #     input_omega_tb2 = w_tb2[-1] - w_tb2[-2]  
            
        self.U_tb2 = np.array([current_vel_tb2, current_omega_tb2]) 
        
        self.c_tb2 = (np.expand_dims(self.X_tb2, axis=0)).transpose()
        self.dev_tb2 = np.array([0.281, 0.306, 0.001])
        self.v_tb2 = np.diag(self.dev_tb2)
        
        self.V_tb2 = np.concatenate([self.c_tb2, self.v_tb2], axis =1)
        self.C_tb2 = []
        self.d_tb2 = []
        self.mu_tb2 = np.zeros(3)        
        self.sigma_tb2 = np.diag(np.ones(3))         
        self.pred_lb_tb2 = np.ones(3) * 4.5 * -1
        self.pred_ub_tb2 = np.ones(3) * 4.5 
        
        
        
        self.A_tb2 = np.array([[1.0, 0.0, 0.0],
                            [0.0, 1.0, 0.0],
                            [0.0, 0.0, 1.0]])
        
        self.dtm_tb2 = 0.9 #odom time period = 0.03 / no of obs
        
        self.b_tb2 = np.array([[cos(theta)*self.dtm_tb2, 0.0],
                                [sin(theta)*self.dtm_tb2, 0.0],
                                [0.0, 1.0]])
        
        

        init_probstar_tb2 = ProbStar(self.V_tb2, self.C_tb2, self.d_tb2, self.mu_tb2,
                                        self.sigma_tb2, self.pred_lb_tb2, self.pred_ub_tb2)
        
        
        
        self.bu_tb2 = np.matmul(self.b_tb2, self.U_tb2).flatten()
        
        
        
        
        next_prob_star_tb2 = init_probstar_tb2.affineMap(self.A_tb2, self.bu_tb2)
        
        for i in range(5):
            next_prob_star_tb2  = next_prob_star_tb2.affineMap(self.A_tb2, self.bu_tb2)
            new_x =  next_prob_star_tb2.V[0][0]
            new_y = next_prob_star_tb2.V[1][0]
            new_theta = next_prob_star_tb2.V[2][0]
            new_quaternion = quaternion_from_euler(0,0,new_theta)   
            marker.publish_prediction_marker("map", i, name = "pred_robot_tb3_2", cord_x= new_x, cord_y=new_y, 
                                                        cord_z= 0.0, std_x=robot_length,
                                                        std_y = robot_width, std_z = robot_height,
                                                        or_x = new_quaternion[0],or_y = new_quaternion[1],
                                                        or_z=new_quaternion[2],or_w=new_quaternion[3])     
                

      
    

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


publish marker:

# marker.publish_pose_marker( name = "pred_robot", cord_x= initial_probstar_rob.mu[0], cord_y=initial_probstar_rob.mu[1], 
        #                                      cord_z= 0.0, std_x=robot_length,
        #                                      std_y = robot_width, std_z = robot_height,
        #                                      or_x = odom_msg.pose.pose.orientation.x,or_y = odom_msg.pose.pose.orientation.y,
        #                                      or_z=odom_msg.pose.pose.orientation.z,or_w=odom_msg.pose.pose.orientation.w) 
        
 # marker.publish_prediction_marker(i, name = "pred_robot", cord_x= next_prob_star.mu[0], cord_y=next_prob_star.mu[1], 
            #                                  cord_z= 0.0, std_x=robot_length,
            #                                  std_y = robot_width, std_z = robot_height,
            #                                  or_x = odom_msg.pose.pose.orientation.x,or_y = odom_msg.pose.pose.orientation.y,
            #                                  or_z=odom_msg.pose.pose.orientation.z,or_w=odom_msg.pose.pose.orientation.w)     


"""
 
        