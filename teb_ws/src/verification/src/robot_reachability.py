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
            
            
            self.z = np.array([[probstar.mu[0]], [probstar.mu[1]]]) #measurement            
            self.est_probstar_k = probstar.affineMap(self.A)
            print("pred: ", self.est_probstar_k.mu[0], self.est_probstar_k.mu[1])            
            self.P_k = np.dot(self.A, np.dot(self.P, self.A.transpose())) + self.Q  # Predicted state covariance
            
            
            S = np.dot(np.dot(self.H, self.P), self.H.transpose()) + self.R
            self.K = np.dot(np.dot(self.P, self.H.transpose()), np.linalg.inv(S))
            I = np.eye(4)
            self.M = I - np.matmul(self.K, self.H)
            self.N = np.matmul(self.K, self.z).flatten()
            
            self.est_probstar = self.est_probstar_k.affineMap(self.M, self.N)
            print("updated: ", self.est_probstar_k.mu[0], self.est_probstar_k.mu[1])
            self.P = np.matmul((np.eye(self.H.shape[1]) - np.matmul(self.K, self.H)), self.P_k)
            
            return self.est_probstar

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
        
        self.kf = kalmanFilter()
        
        
        # Initialize state variables
        self.last_time = rospy.Time.now()     
        
    def human_pc_callback(self, pose_msg):
        
        
        
        self.current_time = rospy.Time.now().to_sec()
        # print(self.current_time)
        pcl_np = pointcloud2_to_numpy(pose_msg)
        self.pose_x = np.mean(pcl_np[0])
        self.pose_y = np.mean(pcl_np[2])
        
        self.x_std = human_length
        self.y_std = human_width
        
        x.append(self.pose_x)
        y.append(self.pose_y)
        
        # Calculate velocity and heading angle
        if len(x) > 1 and len(y) > 1:
            self.dt = self.current_time - self.prev_time
            # self.dt = 1.0
            # print("dt:", self.dt) #0.14
            self.v_x = (x[-1] - x[-2]) / self.dt
            self.v_y = (y[-1] - y[-2]) / self.dt
        
            self.prev_time = self.current_time
        self.heading_angle = math.atan2(y[-1] - y[-2], x[-1] - x[-2])
        # self.velocity = math.sqrt(self.v_x ** 2 + self.v_y ** 2)
        
        
        
        # initial state probstar:
       
        self.mu_initial_human = np.array([self.pose_x,self.pose_y,self.v_x, self.v_y])
        self.std_initial_human = np.array([self.x_std,self.y_std, self.x_std, self.y_std])
        
        self.U_initial_huamn = np.array([self.velocity, self.heading_angle])
        self.sigma_human = np.diag(np.square(self.std_initial_human))
        self.lb_human = self.mu_initial_human - self.std_initial_human / 2
        self.ub_human = self.mu_initial_human + self.std_initial_human / 2
        
        self.initial_probstar_human = ProbStar(self.mu_initial_human, self.sigma_human, self.lb_human, self.ub_human)
        # print( self.initial_probstar_human)
        current_pose = [self.pose_x, self.pose_y]
        # print("original: ",current_pose)
        
        A_2d = np.array([[1.0, 0.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0, 0.0]])
        
        
        new_star = self.kf.predict_update(self.initial_probstar_human, 0.25)
        # new_star2d = new_star.affineMap(A_2d)
        # print(new_star.mu[0], new_star.mu[1])
        p = []
        print("###############################################################")
        for i in range(4):
            new_star = self.kf.predict_update(new_star, 1.0)
            new_star_2d = new_star.affineMap(A_2d)
            print(new_star_2d.mu)
           
            p.append(new_star_2d)
          
            
            
            # print(new_star.mu[0], new_star.mu[1])
        # plot_probstar(p, show=True)
        print("###############################################################")
            
                
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
        
        vel_x = odom_msg.twist.twist.linear.x
        vel_y = odom_msg.twist.twist.linear.y
        vel_z = odom_msg.twist.twist.linear.z
        
        # vel_rob = math.sqrt(vel_x**2 + vel_y**2 + vel_z**2)
        # print(vel_rob)
        vel_rob = vel_x
        
        
        # vel_rob = 0.26
        
        w_x = odom_msg.twist.twist.angular.x
        w_y = odom_msg.twist.twist.angular.y
        w_z = odom_msg.twist.twist.angular.z
        # omega_rob = math.sqrt(w_x**2 + w_y**2 + w_z**2)
        omega_rob = w_z
    
        
        
        self.U = np.array([vel_rob, omega_rob])
        # print("u: ", self.U)
        
        
        # dt_rob = 1.0 #model_dt = 0.25/10   check dt     0.03
        
        
        
        # A_13 = -1 * vel_rob*sin(theta)*dt_rob
        # A_23 = 1 * vel_rob*cos(theta)*dt_rob
        
       
        
        # self.A_rob = np.array([[1.0, 0.0, A_13],
        #                    [0.0, 1.0, A_23],
        #                    [0.0, 0.0, 1.0]])
        
        # print(self.A_rob)
        
        
        
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
    
        # initial_probstar_rob = ProbStar(self.mu_initial_rob, self.sigma_rob, self.lb_rob, self.ub_rob)
        
        # self.bu = np.matmul(self.b_rob, self.U).flatten()
        
        # print("Robot:")
        
        
        # next_prob_star = initial_probstar_rob.affineMap(self.A_rob, self.bu)
        # print(initial_probstar_rob.mu[0], initial_probstar_rob.mu[1])
        # for i in range(4):
        #     next_prob_star = next_prob_star.affineMap(self.A_rob, self.bu)
        #     self.probstars.append(next_prob_star)            
        #     print("step ", i, ": ", next_prob_star.mu[0], next_prob_star.mu[1])
           
        #____________________________________________________________________________________
        
        
        
          
                
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
 
        