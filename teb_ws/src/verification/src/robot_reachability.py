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

class kalmanFilter_probstar:
    
    def __init__(self):
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
        
    
    def predict(self, probstar, dt):
        
        if isinstance (probstar, ProbStar):
        
            self.pose_x = probstar.mu[0]
            self.pose_y = probstar.mu[1]
            
            self.A = np.array([[1, 0, dt], [0, 1, dt], [0, 0, 1]])
            self.z = np.array([[self.pose_x], [self.pose_y]]) 
            
            T = np.matmul(self.P, self.A.transpose())

            self.p_k = np.matmul(self.A,T) + self.Q
            
            # compute kalman gain : K
            T = np.matmul(self.P_k, self.H.transpose())
            T = np.linalg.inv(np.matmul(self.H, T) + self.R)
            T = np.matmul(self.H.transpose(), T)
            self.K = np.matmul(self.P_k, T)
            
            
            self.I = np.eye(3)
            self.M = self.I - np.matmul(self.K, self.H)
            self.N = np.matmul(self.K, self.z).flatten()
            
            self.x_updated = probstar.affineMap(self.M, self.N)
            
            
            # est_pose_x = self.x_updated.mu[0]
            # est_pose_y = self.x_updated.mu[1]
            
            return self.x_updated
        
        
        
        
        

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
        self.velocity = math.sqrt(self.v_x ** 2 + self.v_y ** 2)
        
        for i in range(5):
        
            # initial state probstar:
            self.mu_initial_human = np.array([self.pose_x,self.pose_y,self.heading_angle])
            self.std_initial_human = np.array([self.x_std,self.y_std, 0.01])
            self.U_initial_huamn = np.array([self.velocity, self.heading_angle])
            self.sigma_human = np.diag(np.square(self.std_initial_human))
            self.lb_human = self.mu_initial_human - self.std_initial_human / 2
            self.ub_human = self.mu_initial_human + self.std_initial_human / 2
            
            self.initial_probstar_human = ProbStar(self.mu_initial_human, self.sigma_human, self.lb_human, self.ub_human)
            current_pose = [self.pose_x, self.pose_y]
            print(current_pose)
            
            kf = kalmanFilter_probstar()
            self.next_probstar = kf.predict(self.initial_probstar_human, self.dt)
            
            
            for i in range(5):
                self.next_probstar = kf.predict(self.next_probstar, self.dt)
                new_pose = [self.next_probstar.mu[0], self.next_probstar.mu[1]]
                print("prediction ", i , ": ", new_pose)
                
                
            
            
            
            #-----------------------------------------------------------------------------------------------------------------------------------
            
            
            
            # self.initial_probstar_human = ProbStar(self.mu_initial_human, self.sigma_human, self.lb_human, self.ub_human)
            # self.A = np.array([[1, 0, self.dt], [0, 1, self.dt], [0, 0, 1]])
            # self.z = np.array([[self.pose_x], [self.pose_y]]) 
            
            # T = np.matmul(self.P, self.A.transpose())

            # self.p_k = np.matmul(self.A,T) + self.Q
            
            # # compute kalman gain : K
            # T = np.matmul(self.P_k, self.H.transpose())
            # T = np.linalg.inv(np.matmul(self.H, T) + self.R)
            # T = np.matmul(self.H.transpose(), T)
            # self.K = np.matmul(self.P_k, T)
            
            
            # self.I = np.eye(3)
            # self.M = self.I - np.matmul(self.K, self.H)
            # self.N = np.matmul(self.K, self.z).flatten()
            
            # self.x_updated = self.initial_probstar_human.affineMap(self.M, self.N)
            
            
            # est_pose_x = self.x_updated.mu[0]
            # est_pose_y = self.x_updated.mu[1]
            
            # print("Human:")
            # print("original pose:", [self.pose_x],[self.pose_y]) 
            # print("estimated pose:", [est_pose_x], [est_pose_y])  
            # print()
            # self.pose_x = est_pose_x
            # self.pose_y = est_pose_y
        
        # # Predict next 5 positions
        # for i in range(5):
        #     # self.x = np.matmul(self.A, self.x)
        #     # self.z = np.matmul(self.A, self.z)
        #     # Update the state using the motion model (A) and the current state (z)
        #     # self.z = np.matmul(self.A[:2, :2], self.z) + self.A[:2, 2:]
    
        #     # Update the Kalman filter parameters (M, N) based on the updated state
        #     self.M = self.I - np.matmul(self.K, self.H)
        #     self.N = np.matmul(self.K, self.z).flatten()
            
        #     # Use the updated state to predict the next position
        #     # self.x_updated = self.initial_probstar_human.affineMap(self.M, self.N)
        #     self.x_updated = self.x_updated.affineMap(self.M, self.N)
        #     print(f"Predicted position {i+1}: {self.x_updated.mu[0]}, {self.x_updated.mu[1]}")
        
        # print(self.A)
        # print(self.K)        
        # print("x:", self.x.shape) 
        # print("P:", self.P.shape)
        # print("z:", self.z.shape) 
        # print("A:", self.A.shape)
        # print("x_k:", self.x_k.shape) 
        # print("H:", self.H.shape)  
        # print("R:", self.R.shape)
        # print("Q:", self.Q.shape)    
        
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
        _, _, theta = euler_from_quaternion(quaternion)               
    
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
        
        dt_rob = 1.0 #model_dt = 0.25/10   check dt     0.03
        
        
        
        A_13 = -1 * vel_rob*sin(theta)*dt_rob
        A_23 = 1 * vel_rob*cos(theta)*dt_rob
        
        # print("-vsin0dt: ",A_13)
        # print("vcos0dt: ", A_23)
        
        self.A_rob = np.array([[1.0, 0.0, A_13],
                           [0.0, 1.0, A_23],
                           [0.0, 0.0, 1.0]])
        
        
        
        self.mu_initial_rob = self.X
        self.std_initial_rob = np.array([0.281, 0.306, 0.001]) 
        self.sigma_rob = np.diag(np.square(self.std_initial_rob))
        self.U_initial_rob = self.U      
        self.lb_rob = self.mu_initial_rob - self.std_initial_rob / 2
        self.ub_rob = self.mu_initial_rob + self.std_initial_rob / 2
        
        
        self.A_rob2= np.array([[1.0, 0.0, 0.0],
                           [0.0, 1.0, 0.0],
                           [0.0, 0.0, 1.0]])
        
        self.b_rob = np.array([[cos(theta), 0.0],
                              [sin(theta), 0.0],
                              [0.0, 1.0]])
    
        initial_probstar_rob = ProbStar(self.mu_initial_rob, self.sigma_rob, self.lb_rob, self.ub_rob)
        
        # next_probstar_rob = initial_probstar_rob.affineMap(self.A_rob, self.b)
        print("Robot:")
        
       
        
        # for i in range(10):
        #     next_probstar_rob = next_probstar_rob.affineMap(self.A_rob)
        #     self.probstars.append(next_probstar_rob)
            
        
        self.bu = np.matmul(self.b_rob, self.U).flatten()
        
        # next_prob_star = initial_probstar_rob.affineMap(self.A_rob2, self.bu)
        # print(initial_probstar_rob.mu[0], initial_probstar_rob.mu[1])
        # for i in range(15):
        #     next_prob_star = next_prob_star.affineMap(self.A_rob)
        #     # print(self.A_rob)
        #     self.probstars.append(next_prob_star)            
        #     print("step ", i, ": ", next_prob_star.mu[0], next_prob_star.mu[1])
           
        #____________________________________________________________________________________
        
        plant = DLODE(self.A_rob2, self.b_rob)
        k = 3
        U0 = []
        for i in range(0, k):
            U0.append(self.U)
        # print(U0)
        X, Y = plant.multiStepReach(initial_probstar_rob, U0, k)
        # print('X = {}'.format(X))
        # print('Y = {}'.format(Y))
        
          
                
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
 
        