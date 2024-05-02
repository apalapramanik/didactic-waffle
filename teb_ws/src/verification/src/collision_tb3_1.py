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
from verification.msg import position


from StarV.plant.dlode import DLODE
from StarV.set.probstar import ProbStar



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

prev_time = 0.0
dt = 0.0

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
        collision_probability = intersection.estimateProbability()
        return collision_probability
    
    else:
        return("Error: Input is not a probstar")
    

class robot_human_state:
    
    def __init__(self):
        
        rospy.init_node('tb3_1_collision_node', anonymous=True)
        
         
        self.odom_sub2 = rospy.Subscriber('tb3_1/odom', Odometry, self.odom_callback_tb3_1,queue_size=10)
        
        
        self.pc_human_sub = rospy.Subscriber("tb3_1/position_h1",position,self.human1_pc_callback,queue_size=10)
        
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
      
         
        
    def human1_pc_callback(self, pose_msg):       
        
        
        self.current_time = rospy.Time.now().to_sec()
        
        if not hasattr(self, 'prev_time'):
            self.prev_time = self.current_time
        
        self.dt = (self.current_time - self.prev_time)
        self.prev_time = self.current_time       
        
        self.pose_x = pose_msg.x
        self.pose_y = pose_msg.z
        

        x.append(self.pose_x)
        y.append(self.pose_y)

        # Calculate velocities if there are enough data points
        if len(x) > 1 and len(y) > 1 and self.dt != 0:
            self.v_x = (x[-1] - x[-2]) / self.dt
            self.v_y = (y[-1] - y[-2]) / self.dt
        else:
            self.v_x = 0.0
            self.v_y = 0.0
            
        # print(self.v_x, self.v_y)
        self.dt = round(self.dt, 2)
        # print("dt:", self.dt)
        # print("vx:", self.v_x)
        # print("vy:", self.v_y)

        self.z = np.array([[self.pose_x], [self.pose_y]])
        self.x_human = np.array([[self.z[0, 0]], [self.z[1, 0]], [self.v_x], [self.v_y]])
        print(self.x_human)
        
        #initial probstar  
        self.c_human = self.x_human
        self.dev_human = np.array([human_length, human_width, 0.001, 0.001])    
        self.v_human = np.diag(self.dev_human)
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
            next_probstar_human = kf.predict_update(next_probstar_human,self.dt)
          
            new_x =  next_probstar_human.V[0][0] + (self.z[0]*next_probstar_human.V[0][1]) + (self.v_x * next_probstar_human.V[0][3])
            new_y = next_probstar_human.V[1][0] + (self.z[1]*next_probstar_human.V[1][2]) + (self.v_y * next_probstar_human.V[1][4])
            print("pose ", i ,": ",new_x[0], new_y[0])
           
            marker.publish_prediction_marker("tb3_1_tf/camera_rgb_optical_frame",i, name = "tb3_1_pred_human1", cord_x= new_x[0], cord_y=new_y[0], 
                                                        cord_z= 0.0, std_x=0.5,
                                                        std_y = 0.5, std_z = 0.5,
                                                        or_x = 1.0,or_y =1.0,
                                                        or_z=0.0,or_w=0.0)  
            
      
        
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
  
   
    

if __name__ == '__main__':
   
    robot_state_calc = robot_human_state()
    rospy.spin()
    
    
  
      

 
        