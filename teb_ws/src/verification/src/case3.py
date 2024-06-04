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
import tf2_ros
from tf2_geometry_msgs import do_transform_pose, do_transform_vector3
from math import cos, sin
import math
import csv
from geometry_msgs.msg import Vector3,Vector3Stamped


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

class kalmanFilter:
    
    def __init__(self):
        
        self.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]]) #2x4
        self.Q = np.diag([1.0, 1.0, 1.0, 1.0]) #4x4
        self.R = np.diag([1.0, 1.0]) #2x2
        
       
        self.z = np.array([[0.0],[0.0]])
        self.P   = np.array([[0.0,0.0,0.0, 0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0]])
        self.P_k = np.array([[0.0,0.0,0.0, 0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0]])
        
        
    def predict_update(self, probstar, dt):
        
        if isinstance(probstar, ProbStar):
            
            # Process model
            self.A = np.array([[1, 0, 3*dt, 0],
                                [0, 1, 0, 3*dt],
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]])
            
            self.x_ps = probstar
            
            
            #update error covariance p_k
            self.x_k_ps = self.x_ps.affineMap(self.A)
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
     
        
        return intersection, collision_probability
    
    else:
        return("Error: Input is not a probstar")
    

class robot_human_state:
    
    def __init__(self):
        
        rospy.init_node('tb3_0_collision_node', anonymous=True)
    
        
        self.odom_sub0 = rospy.Subscriber('tb3_0/odom', Odometry, self.odom_callback_tb3_0,queue_size=100)
        self.pc_human_sub = rospy.Subscriber("tb3_0/position_h1",position,self.human1_pc_callback,queue_size=10)
        self.flag_sub = rospy.Subscriber("tb3_0/cp_flag", String, self.cp_flag_callback_tb3_0, queue_size=10 )
        

         
        self.prev_time = 0.0
        # self.dt = 0.0
        
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
        
 
        self.flag = "no"
        
    def cp_flag_callback_tb3_0(self, msg):
        self.flag = msg.data
      
    def human1_pc_callback(self, pose_msg):   
        
        
        self.current_time = rospy.Time.now().to_sec() 
        
        self.dt = 0.2  
        # print("dt:", self.dt)
        self.prev_time = self.current_time
        self.pose_x = pose_msg.x
        self.pose_y = pose_msg.z
        x.append(self.pose_x)
        y.append(self.pose_y)

        # Calculate velocities if there are enough data points
        if len(x) > 1 and len(y) > 1 :
            
            self.v_x = (x[-1] - x[-2]) / self.dt
            self.v_y = (y[-1] - y[-2]) / self.dt
        else:
            self.v_x = 0.0
            self.v_y = 0.0
            
        
        self.v_x = 0.0
        self.v_y = 0.0
        
        self.z = np.array([[self.pose_x], [self.pose_y]])
        self.x_human = np.array([[self.z[0, 0]], [self.z[1, 0]], [self.v_x], [self.v_y]])
        
        
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
        
        self.init_probstar_human = ProbStar(self.V_human, self.C_human, self.d_human,self.mu_human, 
                                       self.sigma_human, self.pred_lb_human,self.pred_ub_human)
        
        marker.publish_pose_marker("tb3_0_tf/camera_rgb_optical_frame", name = "tb3_0_pred_human1", cord_x= self.pose_x, cord_y=0.0, 
                                                        cord_z= self.pose_y, std_x=0.5,
                                                        std_y = 0.5, std_z = 0.5,
                                                        or_x = 1.0,or_y =1.0,
                                                        or_z=0.0,or_w=0.0)   
        
        self.dtp = 0.9
        kf = kalmanFilter()        
        next_probstar_human = kf.predict_update(self.init_probstar_human, self.dtp)  
        # print("human: ",next_probstar_human.V)
        for i in range(4):
            next_probstar_human = kf.predict_update(next_probstar_human,self.dt)
            self.probstars_human.append(next_probstar_human)
            new_x =  next_probstar_human.V[0][0]
            new_y = next_probstar_human.V[1][0]
            # print("human: ", next_probstar_human.V)
           
            marker.publish_prediction_marker("tb3_0_tf/camera_rgb_optical_frame",i, name = "tb3_0_pred_human1", cord_x= new_x, cord_y=0.0, 
                                                        cord_z= new_y, std_x=0.5,
                                                        std_y = 0.5, std_z = 0.5,
                                                        or_x = 1.0,or_y =1.0,
                                                        or_z=0.0,or_w=0.0)    
            
         
    
    def odom_callback_tb3_0(self, odom_msg):
        
         # Create a TransformListener
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)

        # Wait for the transformation between map and camera_rgb_optical_frame
        while not tf_buffer.can_transform("map", "tb3_0_tf/camera_rgb_optical_frame", rospy.Time(), rospy.Duration(1.0)):
            rospy.logwarn_throttle(1.0, "Waiting for transform from map to tb3_0_tf/camera_rgb_optical_frame")

        try:
            # Transform the odometry message pose
            transformed_odom_pose = do_transform_pose(odom_msg.pose, tf_buffer.lookup_transform("tb3_0_tf/camera_rgb_optical_frame", "map", rospy.Time(0)))

            # Create Vector3Stamped messages for linear and angular components
            linear_vector = Vector3Stamped()
            linear_vector.header = odom_msg.header
            linear_vector.vector = odom_msg.twist.twist.linear

            angular_vector = Vector3Stamped()
            angular_vector.header = odom_msg.header
            angular_vector.vector = odom_msg.twist.twist.angular

            # Transform the linear and angular vectors
            transformed_odom_linear = do_transform_vector3(linear_vector, tf_buffer.lookup_transform("map", "tb3_0_tf/camera_rgb_optical_frame", rospy.Time(0)))
            transformed_odom_angular = do_transform_vector3(angular_vector, tf_buffer.lookup_transform("map", "tb3_0_tf/camera_rgb_optical_frame", rospy.Time(0)))

            # # Print or use the transformed odometry message
            # rospy.loginfo("Transformed Odometry Pose: %s", transformed_odom_pose)
            # rospy.loginfo("Transformed Odometry Linear: %s", transformed_odom_linear.vector)
            # rospy.loginfo("Transformed Odometry Angular: %s", transformed_odom_angular.vector)
        except tf2_ros.TransformException as e:
            rospy.logerr("Transform failed: %s" % e)
        
        # Extract pose and twist information from odometry message
        x = transformed_odom_pose.pose.position.x
        y = transformed_odom_pose.pose.position.y
        quaternion = (
            transformed_odom_pose.pose.orientation.x,
            transformed_odom_pose.pose.orientation.y,
            transformed_odom_pose.pose.orientation.z,
            transformed_odom_pose.pose.orientation.w
        )
        
        current_vel_tb0 = transformed_odom_linear.vector.x
        current_omega_tb0 = transformed_odom_angular.vector.z 
     
       
        # # Extract pose and twist information from odometry message
        # x = odom_msg.pose.pose.position.x
        # y = odom_msg.pose.pose.position.y
        # quaternion = (
        #     odom_msg.pose.pose.orientation.x,
        #     odom_msg.pose.pose.orientation.y,
        #     odom_msg.pose.pose.orientation.z,
        #     odom_msg.pose.pose.orientation.w
        # )
        
        # current_vel_tb0 = odom_msg.twist.twist.linear.x
        # current_omega_tb0 = odom_msg.twist.twist.angular.z 
    
        
        _, _, theta = euler_from_quaternion(quaternion)  
        # theta = 0.5             
    
        self.X_tb0 = np.array([x, y, theta])  
     
  
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
        
        self.dtm_tb0 = 0.7 
        
        self.b_tb0 = np.array([[cos(theta)*self.dtm_tb0, 0.0],
                              [sin(theta)*self.dtm_tb0, 0.0],
                              [0.0, 1.0]])
    
        init_probstar_tb0 = ProbStar(self.V_tb0, self.C_tb0, self.d_tb0, self.mu_tb0,
                                     self.sigma_tb0, self.pred_lb_tb0, self.pred_ub_tb0)     
        
        
        self.bu_tb0 = np.matmul(self.b_tb0, self.U_tb0).flatten()
        
        next_prob_star_tb0 = init_probstar_tb0.affineMap(self.A_tb0, self.bu_tb0)
        p_set = []
        self.probstars_tb3_0 = []
        print("robot: ", next_prob_star_tb0.V)
        for i in range(4):
            next_prob_star_tb0  = next_prob_star_tb0.affineMap(self.A_tb0, self.bu_tb0)
            self.probstars_tb3_0.append(next_prob_star_tb0)
            new_x =  next_prob_star_tb0.V[0][0]
            new_y = next_prob_star_tb0.V[1][0]
            new_theta = next_prob_star_tb0.V[2][0]
            new_quaternion = quaternion_from_euler(0,0,new_theta)
            # print("robot: ", next_prob_star_tb0.V)
            # marker.publish_prediction_marker("tb3_0_tf/camera_rgb_optical_frame", i, name = "pred_robot_tb3_0", cord_x= new_x, cord_y=0.0, 
            #                                             cord_z= new_y, std_x=robot_length,
            #                                             std_y = robot_width, std_z = robot_height,
            #                                             or_x = 1.0,or_y =1.0 ,
            #                                             or_z=0.0,or_w=0.0) 
            
            
            if (self.flag == "no" ):
                print("no human found")
            else:
                
                ps, p = probstar_halfspace_intersection_2d( self.probstars_tb3_0[i],self.init_probstar_human)                
                p_set.append(p)
                
                if (p>0):
                    print("probability:", p*100)
                    # print("--------------------------------------------------------------")
                    print("tb0 probstar:",self.probstars_tb3_0[i].V )
                    # print("--------------------------------------------------------------")
                    print("human probstar: ", self.init_probstar_human.V)
                    
                    print("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^")
                    print()
                    # print(ps)
                
                else:
                    print("nopee")
                    p = 0
                    p_set.append(p)     
            
        
        
       
        print("...........................................................................")
        
      
                
                
        
   
       
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
 
        