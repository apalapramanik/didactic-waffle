import roslib
import rospy
import numpy as np
from std_msgs.msg import Header
from std_msgs.msg import Float64
from random import gauss
from sensor_msgs.msg import PointCloud2 as pc2
x = []
y = []

class kf_probstar:
    
    def __init__(self):
        self.pc_human_sub = rospy.Subscriber("projected",pc2,self.human_pc_callback,queue_size=10)
        self.prev_time = None
        
        
        self.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
        self.Q = np.diag([0.01, 0.01, 1.0, 1.0])
        self.R = np.diag([0.01, 1.0])
        
        self.x = np.array([[0.0],[0.0],[0.0],[0.0]])
        self.z = np.array([[0.0],[0.0]])
        self.u = 0
        self.P   = np.array([[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]])
        self.P_k = np.array([[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]])
         
         
         
    def human_pc_callback(self, pose_msg):
            
            current_time = rospy.Time.now()
            pcl_np = pointcloud2_to_numpy(pose_msg)
            x.append(pcl_np[0])
            y.append(pcl_np[1])
            
            # Calculate velocities if there are enough data points
            if len(self.x) > 1 and len(self.y) > 1:
                if self.prev_time is not None:
                    self.dt = current_time - self.prev_time
                    self.v_x = (self.x[-1] - self.x[-2]) / self.dt
                    self.v_y = (self.y[-1] - self.y[-2]) / self.dt
                    self.prev_time = current_time
                    
            # self.dt = 0.25 #check time
        
            self.A = np.array([[1, 0, self.dt, 0], [0, 1, 0, self.dt], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
            
        
            
            
            self.z = np.array([pcl_np[0]], [pcl_np[1]])              
            self.x - np.array([self.z[0], self.z[1], self.v_x, self.v_y])
            
            self.x_k = np.matmul(self.A, self.x)
            T = np.matmul(self.P, self.A.transpose())
            self.p_k = np.matmul(self.A, T) + self.Q
            
            T = np.matmul(self.P_k, self.H.transpose())
            T = np.linalg.inv(np.matmul(self.H, T) + self.R)
            T = np.matmul(self.H.transpose(), T)
            self.K = np.matmul(self.P_k, T)
            
            self.x = self.x_k + np.matmul(self.K, (self.z - np.matmul(self.H, self.x_k)))
            self.P = np.matmul((np.eye(self.H.shape[1]) - np.matmul(self.K, self.H)), self.P_k)
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
            
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
    human_state_calc.estimate()
    rospy.spin()
        

