from eval_pred import *
import numpy as np
import math
import rospy
from sensor_msgs.msg import PointCloud2 as pc2
from std_msgs.msg import String
import quaternion # https://github.com/moble/quaternion
from nav_msgs.msg import Odometry
from marker import marker
import ros_numpy

from cv2 import HuMoments
import rospy
from array import array
from cmath import isnan, nan, sqrt
from os import device_encoding
from numpy import NaN, cov, poly
from sklearn.cluster import DBSCAN

import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection
import seaborn as sns



filter_type = 'kalman'
steps = 5

odom_pose = []
odom_or = []
rot_array = []
trans_array = []


class predict:
    def __init__(self):
        
        #subscribers
        rospy.Subscriber("projected",pc2,self.cloud_callback,queue_size=10)  
        rospy.Subscriber("cp_flag", String, self.cp_flag_callback, queue_size=10 )
        rospy.Subscriber("odom", Odometry, self.odom_callback,queue_size=10)
        
    def cp_flag_callback(self, msg):
        self.flag = msg.data
        
   
    def odom_callback(self, data):
        
        # unpack pose x,y,z
        self.curr_pose_x = data.pose.pose.position.x
        self.curr_pose_y = data.pose.pose.position.y
        self.curr_pose_z = data.pose.pose.position.z
        
        # unpack orientation x,y,z,w
        self.curr_or_x = data.pose.pose.orientation.x
        self.curr_or_y = data.pose.pose.orientation.y
        self.curr_or_z = data.pose.pose.orientation.z
        self.curr_or_w = data.pose.pose.orientation.w
        
   
        # append current and previous pose,orientation in array
        if len(odom_pose)<2 :
                odom_pose.append([self.curr_pose_x, self.curr_pose_y,self.curr_pose_z])
                odom_or.append([self.curr_or_x, self.curr_or_y,self.curr_or_z,self.curr_or_w])
                
        else :
            
            odom_pose.pop(0)
            odom_pose.append([self.curr_pose_x, self.curr_pose_y,self.curr_pose_z])
            odom_or.pop(0)
            odom_or.append([self.curr_or_x, self.curr_or_y,self.curr_or_z,self.curr_or_w])
    
        
        # convert orientation into quaternion 
        q_old = np.quaternion(odom_or[0][3], odom_or[0][0], odom_or[0][1], odom_or[0][2])
        p_old = [odom_pose[0][0], odom_pose[0][1], odom_pose[0][2]]
        
        q_new = np.quaternion(odom_or[1][3], odom_or[1][0], odom_or[1][1], odom_or[1][2])
        p_new = [odom_pose[1][0], odom_pose[1][1], odom_pose[1][2]]
        
        #calculate rotation quaternion: q_rotation = q_current * ((q_previous)transpose)
        q_old_inv = np.conjugate(q_old)
        global q_rot 
        q_rot = q_new * q_old_inv
        
        # compute translation matrix : x, y, z
        translation_x = odom_pose[1][0] - odom_pose[0][0]
        translation_y = odom_pose[1][1] - odom_pose[0][1]
        translation_z = odom_pose[1][2] - odom_pose[0][2]

        global translation
        translation = [translation_x, translation_y, translation_z]
        
        #save 10 rotations and translations in array
        if len(rot_array)<10:
            rot_array.append(q_rot)
            trans_array.append(translation)
        else:
            rot_array.pop(0)
            rot_array.append(q_rot)
            trans_array.pop(0)
            trans_array.append(translation)
            
    


            
        
    def cloud_callback(self, data):
        if self.flag == 'yes':
            
            #convert point cloud to numpy array:
            pcl_np = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(data, remove_nans=False) 
            
        
            #create a 2D array out of this pcl_np without the y values which are 0 after projection
            xzarray = []
            height, width = pcl_np.shape
            x_values = []
            z_values = []
            for i in range(0,height):
                    point = pcl_np[i]
                    x = point[0]
                    z = point[2]
                    x_values.append(x)
                    z_values.append(z)
                    value = [x,z]
                    xzarray.append(value)
            
        
            
            # find mean and covariance for the XZ array:
            xz_np_array = np.array(xzarray)
            mean2D = xz_np_array.mean(axis=1)
            cov_xz = np.cov(xz_np_array)      
            
            # compute DBSCAN - change eps and min_samples as required, eps: min distance between points
            # learn more from - https://scikit-learn.org/stable/modules/generated/sklearn.cluster.DBSCAN.html
            
            
            
            #start db scan:
            DBSCAN_cluster = DBSCAN(eps=0.5, min_samples=30).fit(xz_np_array) #0.5, 30
            labels = DBSCAN_cluster.labels_
            components = DBSCAN_cluster.components_ #copy of each core sample found by training
            # feature = DBSCAN_cluster.n_features_in_ #number of features seen during fit
            
        
            
            for i in range(len(components)):
                
                meanx, meanz, transform_array = transform(i, f'useful_cluster{i}', f'x_array{i}', f'z_array{i}',
                                                          f'transform_array_{i}', components, f'position_msg{i}', f'pose_pub{i}' )
                
                filter_estimator = FilterEstimator(transform_array, steps)
                filter_estimator.main(filter_type)
                
                
                marker.publish_human_marker(f'human{i}', cord_x = meanx, cord_y = 0.0, cord_z = meanz)
                    
                
                    
                    
def transform(i, cluster_name, x_array, z_array, transform_array, components, position_msg, pose_pub):
    cluster_name.append(components[i])
    point = components[i] 
    x_array.append(point[0])
    z_array.append(point[1])

    #get mean of cluster for human position:
    meanx = np.nanmean(x_array)
    meanz = np.nanmean(z_array)
    
    #get x,z position cordinates for kf:
    pos1 = [meanx,meanz,0.0] 
    # human1_array.append(pos1)
    # np.savetxt("org1.txt", human1_array, delimiter=",")
    

    #add position to array and transform:
    if len(transform_array)<15:
        if len(transform_array)==0:
            transform_array.append(pos1)
            
        else:
            for i in range(len(transform_array)):
                transform_array[i] = quaternion.rotate_vectors(q_rot,transform_array[i]) + translation
            transform_array.append(pos1)
        
    else:
        transform_array.pop(0)
        for i in range(len(transform_array)):
                transform_array[i] = quaternion.rotate_vectors(q_rot,transform_array[i]) + translation
        transform_array.append(pos1)
        
    # publish current human position
    # position_msg.header = data.header
    position_msg.x = meanx #pos1_trans[0] #meanx1
    position_msg.z = meanz #pos1_trans[1]#meanz1
    pose_pub.publish(position_msg)
    
    return meanx, meanz, transform_array