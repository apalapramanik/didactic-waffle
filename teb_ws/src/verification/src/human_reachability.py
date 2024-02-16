#!/usr/bin/env python

"""
    Author:  Apala Pramanik 
    Project: Autonomous Robot Safety Verification in Construction site
    Advised by: Dr.Dung Hoang Tran    
    
"""



import rospy
import numpy as np
from std_msgs.msg import Header
from std_msgs.msg import Float64
from random import gauss
from sensor_msgs.msg import PointCloud2 as pc2
import math
from StarV.set.probstar import ProbStar

x = []
y = []
# received = False
human_length = 1.79 #avg length (full arm) for men
human_width = 1.79 #avg width (full arm) for men
human_height = 1.740 #avg height for men




class kf_probstar:
    
    def __init__(self):
        
        rospy.init_node('kf')
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
            # print("dt:", self.dt) #0.14
            self.v_x = (x[-1] - x[-2]) / self.dt
            self.v_y = (y[-1] - y[-2]) / self.dt
        
            self.prev_time = self.current_time
        self.heading_angle = math.atan2(y[-1] - y[-2], x[-1] - x[-2])
        self.velocity = math.sqrt(self.v_x ** 2 + self.v_y ** 2)
            
        
        # initial state probstar:
        self.mu_initial_human = np.array([self.pose_x,self.pose_y,self.heading_angle])
        self.std_initial_human = np.array([self.x_std,self.y_std, 0.01])
        self.U_initial_huamn = np.array([self.velocity, self.heading_angle])
        self.sigma_human = np.diag(np.square(self.std_initial_human))
        self.lb_human = self.mu_initial_human - self.std_initial_human / 2
        self.ub_human = self.mu_initial_human + self.std_initial_human / 2
        
        self.initial_probstar_human = ProbStar(self.mu_initial_human, self.sigma_human, self.lb_human, self.ub_human)
        self.A = np.array([[1, 0, self.dt], [0, 1, self.dt], [0, 0, 1]])
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
        
        self.x_updated = self.initial_probstar_human.affineMap(self.M, self.N)
        
        
        est_pose_x = self.x_updated.mu[0]
        est_pose_y = self.x_updated.mu[1]

     
        
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
        
        print("original pose:", [self.pose_x],[self.pose_y])   

        print("estimated pose:", [est_pose_x], [est_pose_y])
            


          
            
            
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
   
    human_state_calc = kf_probstar()
    # human_state_calc.predict()
   
    rospy.spin()
    
        
