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
from tf.transformations import euler_from_quaternion
import numpy as np
from scipy.linalg import expm
from sensor_msgs.msg import PointCloud2 as pc2
from math import cos, sin
import math
# from probstar import ProbStar

from StarV.plant.dlode import DLODE
from StarV.set.probstar import ProbStar
from plot import plot_probstar, plot_star
# from StarV.util.plot import probstar2polytope, plot_probstar

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
            self.A = np.array([[1, 0, 1, dt],
                                [0, 1, 0, dt],
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
            
            print("Estimated pose:", self.x_ps.mu[0], self.x_ps.mu[1])
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
 
    def publish_pose_marker(name, cord_x, cord_y, cord_z, std_x, std_y, std_z, or_x, or_y, or_z, or_w):
        
        human_marker = rospy.Publisher(name, Marker, queue_size=0)
        prediction_marker_cube = Marker()
    
        
        prediction_marker_cube.header.stamp = rospy.Time.now()
        prediction_marker_cube.header.frame_id = "map"
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
        
    def publish_prediction_marker(a, name, cord_x, cord_y, cord_z, std_x, std_y, std_z, or_x, or_y, or_z, or_w):
       
        prediction_marker = rospy.Publisher(name, Marker, queue_size=0)
        pred_marker_cube = Marker()
        
        pred_marker_cube.header.stamp = rospy.Time.now()
        pred_marker_cube.header.frame_id = "map"
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
        
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback,queue_size=10) 
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
        self.dt = self.current_time - self.prev_time
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
            
        print(self.v_x, self.v_y)
        self.dt = round(self.dt, 2)
        print(self.dt)

        self.z = np.array([[self.pose_x], [self.pose_y]])
        self.x = np.array([[self.z[0, 0]], [self.z[1, 0]], [self.v_x], [self.v_y]])
        print(self.x)
        
        #initial probstar        
        self.mu = np.array([self.z[0, 0], self.z[1, 0], self.v_x, self.v_y])
        self.std = np.array([human_length, human_width, 0.1, 0.1])
        self.sigma = np.diag(np.square(self.std))
        self.lb = self.mu - self.std / 2
        self.ub = self.mu + self.std / 2
        
        self.init_probstar_human = ProbStar(self.mu, self.sigma, self.lb, self.ub)
        
        kf = kalmanFilter()
        
        self.next_prob_star_human = kf.predict_update(self.init_probstar_human, self.dt)
        
        print("Human:")
        
        for i in range(5):
            self.next_prob_star_human = kf.predict_update(self.next_prob_star_human, self.dt)
            
            
                
        #--------------------------------------------------------------------------------------------------------------------------------------


         
   
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
        # _, _, theta = euler_from_quaternion(quaternion)  
        theta = 0.5             
    
        self.X = np.array([x, y, theta])  
     
        vel_rob = odom_msg.twist.twist.linear.x      
        omega_rob = odom_msg.twist.twist.angular.z  
        self.U = np.array([vel_rob, omega_rob])
        # print("u: ", self.U)        
        
        self.mu_initial_rob = self.X
        self.std_initial_rob = np.array([0.281, 0.306, 0.001]) 
        self.sigma_rob = np.diag(np.square(self.std_initial_rob))
        self.U_initial_rob = self.U      
        self.lb_rob = self.mu_initial_rob - self.std_initial_rob / 2
        self.ub_rob = self.mu_initial_rob + self.std_initial_rob / 2
        
        
        self.A_rob= np.array([[1.0, 0.0, 0.0],
                           [0.0, 1.0, 0.0],
                           [0.0, 0.0, 1.0]])
        
        self.b_rob = np.array([[cos(theta), 0.0],
                              [sin(theta), 0.0],
                              [0.0, 1.0]])
        
        # print("b: ", self.b_rob)
    
        init_probstar_rob = ProbStar(self.mu_initial_rob, self.sigma_rob, self.lb_rob, self.ub_rob)
        
        self.bu = np.matmul(self.b_rob, self.U).flatten()
        
        print("Robot:")
        
        
        next_prob_star_rob = init_probstar_rob.affineMap(self.A_rob, self.bu)
        
        for i in range(4):
            next_prob_star_rob  = next_prob_star_rob.affineMap(self.A_rob, self.bu)
     
        
        
        
          
                
        print("---------------------------------------------------")
    

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



DLODE:...print('\nTesting DLODE multiStepReach method....')
            k = 3
            X0 = ProbStar.rand(2)
            plant = DLODE.rand(2, 2, 1)
            plant.info()
            #U0 = np.random.rand(10, 2)
            U0 = []
            for i in range(0, k):
                U0.append(ProbStar.rand(2))
            X, Y = plant.multiStepReach(X0, U0, k)
            print('X = {}'.format(X))
            print('Y = {}'.format(Y))
"""
 
        