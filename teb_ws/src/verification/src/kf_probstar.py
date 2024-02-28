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
from StarV.plant.dlode import DLODE
from StarV.set.probstar import ProbStar
from plot import plot_probstar, plot_star


x = []
y = []
# received = False
human_length = 1.79  # avg length (full arm) for men
human_width = 1.79  # avg width (full arm) for men
human_height = 1.740  # avg height for men


class kf_probstar:

    def __init__(self):

        rospy.init_node('kf')
        self.flag = False
        self.pc_human_sub = rospy.Subscriber("projected", pc2, self.human_pc_callback, queue_size=10)
        self.prev_time = rospy.Time.now().to_sec()
        self.dt = 0.0

        self.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
        self.Q = np.diag([0.01, 0.01, 0.01, 1.0])
        self.R = np.diag([0.01, 1.0])

        self.x = np.array([[0.0], [0.0], [0.0], [0.0]])
        self.z = np.array([[0.0], [0.0]])
        self.u = 0
        self.P = np.eye(4)
        self.P_k = np.eye(4)
        self.v_x = 0
        self.v_y = 0
        
    def predict(self, probstar):
        if  isinstance(probstar, ProbStar):
        
            
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

        self.A = np.array([[1, 0, self.dt, 0], [0, 1, 0, self.dt], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
        self.z = np.array([[self.pose_x], [self.pose_y]])
        self.x = np.array([[self.z[0, 0]], [self.z[1, 0]], [self.v_x], [self.v_y]])
        print(self.x)
        
        #initial probstar        
        self.mu = np.array([self.z[0, 0], self.z[1, 0], self.v_x, self.v_y])
        self.std = np.array([human_length, human_width, 0.1, 0.1])
        self.sigma = np.diag(np.square(self.std))
        self.lb = self.mu - self.std / 2
        self.ub = self.mu + self.std / 2
        
        self.probstar = ProbStar(self.mu, self.sigma, self.lb, self.ub)
        self.flag = True
        # self.next_probstar = self.predict(self.probstar)
        self.next_probstar = self.probstar.affineMap(self.A)
        
        a = np.array([[1.0, 0.0, 0.0, 0.0],
                      [0.0, 1.0 , 0.0, 0.0]])
        
        p = []
        print("Original pose:", self.pose_x, self.pose_y)
        for i in range(5):
            self.next_probstar = self.next_probstar.affineMap(self.A)
            # print(self.next_probstar.V)
            print("Estimated pose:", self.next_probstar.V[0][3], self.next_probstar.V[1][4])
            # self.ps_2d = self.next_probstar.affineMap(a)
            # p.append(self.ps_2d)
            # plot_probstar(p, show=True)
            
            
        
        
        
        
        
        
        # # Estimate 5 future predictions
        # for i in range(5):
        #     self.x_k_ps= self.x_ps.affineMap(self.A)
        #     T = np.matmul(self.P, self.A.transpose())
        #     self.P_k = np.matmul(self.A, T) + self.Q
            
            
        #     self.x_ps = self.x_k_ps.affineMap(self.M, self.N)
        #     self.P = np.matmul((np.eye(self.H.shape[1]) - np.matmul(self.K, self.H)), self.P_k)

        #     print(f"Estimated pose {i+1} step ahead:", self.x_ps.mu[0], self.x_ps.mu[1])

        
        # print("Estimated pose:", self.next_probstar.mu[0], self.next_probstar.mu[1])
        
        


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
    rospy.spin()
