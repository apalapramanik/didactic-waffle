"""
    Author:  Apala Pramanik 
    Project: Autonomous Robot in Construction site
    Advised by: Dr.Dung Hoang Tran, Dr.Kyungki Kim
    Operations performed: 
            1) Plot gaussian distribution
            2) Implement DB Scan
            3) Kalman filtering for prediction
            4) Visualization of human and prediction in rviz
    
"""

#!/usr/bin/env python
#!/usr/bin/env python3

#imports 
import sys
import time
# from turtle import distance

# from cv_kf import KalmanFilter
from cv2 import HuMoments
import rospy
from array import array
from cmath import isnan, nan, sqrt
from os import device_encoding
from numpy import NaN, cov, poly
import ros_numpy
import quaternion # https://github.com/moble/quaternion

import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection
import seaborn as sns
from sklearn.cluster import DBSCAN
from testrobots.msg import distance
from sensor_msgs.msg import PointCloud2 as pc2
from nav_msgs.msg import Odometry
import cv2


from visualization_msgs.msg import Marker, MarkerArray
from testrobots.msg import position
from testrobots.msg import Plot
import matplotlib.pyplot as plt
from scipy.stats import norm
import warnings
import numpy as np
import time
import math
from timeit import default_timer as timer
from datetime import timedelta
from datetime import timedelta




human1_array = []
human2_array = []
human3_array = []

prev_pred_x1 = []
prev_pred_z1 = []

prev_pred_x2 = []
prev_pred_z2 = []

prev_pred_x3 = []
prev_pred_z3 = []

odom_pose = []
odom_or = []

pred_array1 = []
pred_array2 = []
pred_array3 = []

rot_array = []
trans_array = []

transform_array1 = []
transform_array2 = []
transform_array3 = []



class KalmanFilter:
    
    kf = cv2.KalmanFilter(4, 2)
    kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32) #H
    kf.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32) #A


    def predict(self, coordX, coordY):
        # This function estimates the position of the object
        measured = np.array([[np.float32(coordX)], [np.float32(coordY)]])
        self.kf.correct(measured) #x(k)=x'(k)+K(k)*(z(k)-H*x'(k))
        predicted = self.kf.predict()
        x, y = float(predicted[0]), float(predicted[1])
        return x, y


class Prediction(object):

    def __init__(self):
        
        self.indicator = 0
        rospy.Subscriber("projected",pc2,self.cloud_callback,queue_size=1)  
        rospy.Subscriber("H_Detection_msg",Plot,self.indicator_callback,queue_size=1)  
        rospy.Subscriber("odom", Odometry, self.odom_callback)
        
        #human1   
        self.human_marker1 = rospy.Publisher("Human_marker_h1", Marker, queue_size=0)
        self.prediction_marker1 = rospy.Publisher("Prediction_marker_h1", Marker, queue_size=0)
        self.pose_human1 = rospy.Publisher("position_h1", position,queue_size=1)
        self.pred_human1 = rospy.Publisher("prediction_h1", position,queue_size=1)
      
        
        #human2
        self.human_marker2 = rospy.Publisher("Human_marker_h2", Marker, queue_size=0)
        self.prediction_marker2 = rospy.Publisher("Prediction_marker_h2", Marker, queue_size=0)
        self.pose_human2 = rospy.Publisher("position_h2", position,queue_size=1)
        self.pred_human2 = rospy.Publisher("prediction_h2", position,queue_size=1)
       
        
        #human3
        self.human_marker3 = rospy.Publisher("Human_marker_h3", Marker, queue_size=0)
        self.prediction_marker3 = rospy.Publisher("Prediction_marker_h3", Marker, queue_size=0)
        self.pose_human3 = rospy.Publisher("position_h3", position,queue_size=1)
        self.pred_human3 = rospy.Publisher("prediction_h3", position,queue_size=1)
       
        
    def indicator_callback(self, data) :
        self.indicator = data.value
    
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
    
        
        #  convert orientation into quaternion 
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
            
        
        
       
    def cloud_callback(self,data):
        
        
        
        start = timer()
        if self.indicator == 1:
           
    
            header = data.header
            
            Human_Marker_cube1 = Marker()
            prediction1 = Marker()
            position_msg1 = position()
            prediction_msg1 = position()
          
            
            Human_Marker_cube2 = Marker()
            prediction2 = Marker()
            position_msg2 = position()
            prediction_msg2 = position()
         
            
            Human_Marker_cube3 = Marker()
            prediction3 = Marker()
            position_msg3 = position()
            prediction_msg3 = position()
          
           
            
          
            print("\n ********************************************************************** \n")
            
            
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
            
           
            
    #------------------------------------------- DB SCAN ------------------------------------------------------------------
            
            # compute DBSCAN - change eps and min_samples as required, eps: min distance between points
            # learn more from - https://scikit-learn.org/stable/modules/generated/sklearn.cluster.DBSCAN.html
            
            
            
            #start db scan:
            DBSCAN_cluster = DBSCAN(eps=0.5, min_samples=30).fit(xz_np_array) #0.5, 30
            labels = DBSCAN_cluster.labels_
            components = DBSCAN_cluster.components_ #copy of each core sample found by training
            feature = DBSCAN_cluster.n_features_in_ #number of features seen during fit
            
            # useful cluster has human with label 0, might change if more humans are added
            # x and y are points needed for plotting       
            useful_cluster1 = []
            useful_cluster2 = []
            useful_cluster3 = []
            x1 = []
            z1 = []
            x2 = []
            z2 = []
            x3 = []
            z3 = []
            
            
                
            for i in range(len(components)):
                if labels[i] == 0 :
                    useful_cluster1.append(components[i])
                    point1 = components[i] 
                    x1.append(point1[0])
                    z1.append(point1[1])
                
                    #get mean of cluster for human position:
                    meanx1 = np.nanmean(x1)
                    meanz1 = np.nanmean(z1)
                    
                    #get x,z position cordinates for kf:
                    pos1 = [meanx1,meanz1,0.0] 
                    # human1_array.append(pos1)
                    # np.savetxt("org1.txt", human1_array, delimiter=",")
                    
        
                    #add position to array and transform:
                    if len(transform_array1)<15:
                        if len(transform_array1)==0:
                            transform_array1.append(pos1)
                            
                        else:
                            for i in range(len(transform_array1)):
                                transform_array1[i] = quaternion.rotate_vectors(q_rot,transform_array1[i]) + translation
                            transform_array1.append(pos1)
                        
                    else:
                        transform_array1.pop(0)
                        for i in range(len(transform_array1)):
                                transform_array1[i] = quaternion.rotate_vectors(q_rot,transform_array1[i]) + translation
                        transform_array1.append(pos1)
                
                    # publish current human position
                    position_msg1.header = data.header
                    position_msg1.x = meanx1 #pos1_trans[0] #meanx1
                    position_msg1.z = meanz1 #pos1_trans[1]#meanz1
                    self.pose_human1.publish(position_msg1)
                    
                    #create marker at current mean position of human:
                    Human_Marker_cube1.header.stamp = rospy.Time.now()
                    Human_Marker_cube1.header.frame_id = "camera_rgb_optical_frame"
                    Human_Marker_cube1.ns = "basic_shapes_1"
                    Human_Marker_cube1.id = 1
                    Human_Marker_cube1.type = 1
                    Human_Marker_cube1.pose.position.x = meanx1 #pos1_trans[0] #meanx1 
                    Human_Marker_cube1.pose.position.y = 0.0
                    Human_Marker_cube1.pose.position.z = meanz1 #pos1_trans[1]#meanz1 
                    Human_Marker_cube1.pose.orientation.x = 1.0
                    Human_Marker_cube1.pose.orientation.y = 1.0
                    Human_Marker_cube1.pose.orientation.z = 0.0
                    Human_Marker_cube1.pose.orientation.w = 0.0
                    Human_Marker_cube1.scale.x = 0.7
                    Human_Marker_cube1.scale.y = 0.7
                    Human_Marker_cube1.scale.z = 0.7
                    Human_Marker_cube1.color.a = 1.0
                    Human_Marker_cube1.color.r = 1.0
                    Human_Marker_cube1.color.g = 0.0
                    Human_Marker_cube1.color.b = 0.0
                    
                    #publish marker at current mean position of human:
                    self.human_marker1.publish(Human_Marker_cube1)
                    
                    
                    #start kalman filtering:
                    kf_cv = KalmanFilter()
                    # print(human_pos)
                    
                    
                
                    for pt in transform_array1:
                
                        predicted1 = kf_cv.predict(pt[0], pt[1]) 
                    # pred_array1.append(predicted1)
                    
                    # np.savetxt("pred1.txt", pred_array1, delimiter=",")
                            
                        
                    
                    if math.isnan(pt[0]): #math.isnan(pt[0])
                            print("No Human 1")
                    else:        
                        a = 0                        
                        #repeat kf on prediction :      
                        for j in range(7):  
                            a = a + 1                  
                            predicted1 = kf_cv.predict(predicted1[0],predicted1[1])
                            print("predicted 1:",predicted1) 
                            prediction1.header.frame_id = "camera_rgb_optical_frame"
                            prediction1.ns = "basic_shapes"
                            prediction1.id = a
                            prediction1.type = 1
                            prediction1.pose.position.x =  predicted1[0]
                            prediction1.pose.position.y = 0.0
                            prediction1.pose.position.z = predicted1[1]
                            prediction1.pose.orientation.x = 1.0
                            prediction1.pose.orientation.y = 1.0
                            prediction1.pose.orientation.z = 0.0
                            prediction1.pose.orientation.w = 0.0
                            prediction1.scale.x = 0.7
                            prediction1.scale.y = 0.7
                            prediction1.scale.z = 0.7
                            prediction1.color.a = 1.0
                            prediction1.color.r = 0.0
                            prediction1.color.g = 1.0
                            prediction1.color.b = 0.0
                            
                            #publish marker for predicted position:
                            self.prediction_marker1.publish(prediction1)
                            pred_array1.append(predicted1)
                            # np.savetxt("pred1.txt", pred_array1, delimiter=",")
                            # print(pred_array1)
                            
                            
                            
                            
                            
                        # #publish predicted positions of human:
                        # prediction_msg1.header = data.header
                        # prediction_msg1.x = predicted1[0]
                        # prediction_msg1.z = predicted1[1]
                        # self.pred_human1.publish(prediction_msg1)
                    
                #**************************************************************  
                    
                if labels[i] == 1 :
                    useful_cluster2.append(components[i])
                    point2 = components[i] 
                    x2.append(point2[0])
                    z2.append(point2[1])
                    
                  
                    meanx2 = np.nanmean(x2)
                    meanz2 = np.nanmean(z2)
                    
                    #get x,z position cordinates:
                    pos2 = [meanx2,meanz2,0.0] 
                    # human2_array.append(pos2)
                    # np.savetxt("org2.txt", human2_array, delimiter=",")
                    
                    # add positions to array and transform :
                    if len(transform_array2)<15:
                        if len(transform_array2)==0:
                            transform_array2.append(pos2)
                            
                        else:
                            for i in range(len(transform_array2)):
                                transform_array2[i] = quaternion.rotate_vectors(q_rot,transform_array2[i]) + translation
                            transform_array2.append(pos2)
                            
                    else:
                        transform_array2.pop(0)
                        for i in range(len(transform_array2)):
                                transform_array2[i] = quaternion.rotate_vectors(q_rot,transform_array2[i]) + translation
                        transform_array2.append(pos2)
                    
            
                    #publish current human position
                    position_msg2.header = data.header
                    position_msg2.x = meanx2 #pos2_trans[0] #meanx2
                    position_msg2.z = meanz2 #pos2_trans[1] #meanz2
                    self.pose_human2.publish(position_msg2)
                    
                    #create marker at current mean position of human:
                    Human_Marker_cube2.header.stamp = rospy.Time.now()
                    Human_Marker_cube2.header.frame_id = "camera_rgb_optical_frame"
                    Human_Marker_cube2.ns = "basic_shapes_2"
                    Human_Marker_cube2.id = 2
                    Human_Marker_cube2.type = 1
                    Human_Marker_cube2.pose.position.x = meanx2 #pos2_trans[0] #meanx2 #transformed_pose.position.x
                    Human_Marker_cube2.pose.position.y = 0.0
                    Human_Marker_cube2.pose.position.z = meanz2 #pos2_trans[1] #meanz2 #transformed_pose.position.z
                    Human_Marker_cube2.pose.orientation.x = 1.0
                    Human_Marker_cube2.pose.orientation.y = 1.0
                    Human_Marker_cube2.pose.orientation.z = 0.0
                    Human_Marker_cube2.pose.orientation.w = 0.0
                    Human_Marker_cube2.scale.x = 0.7
                    Human_Marker_cube2.scale.y = 0.7
                    Human_Marker_cube2.scale.z = 0.7
                    Human_Marker_cube2.color.a = 1.0
                    Human_Marker_cube2.color.r = 1.0
                    Human_Marker_cube2.color.g = 0.0
                    Human_Marker_cube2.color.b = 0.0
                    
                    #publish marker at current mean position of human:
                    self.human_marker2.publish(Human_Marker_cube2)
                    
                    #start kalman filtering:
                    kf_cv = KalmanFilter()
                    # print(human_pos)
                
                    for pt in transform_array2: 
                        
                        predicted2 = kf_cv.predict(pt[0], pt[1]) 
                    pred_array2.append(predicted2)
                    # np.savetxt("pred2.txt", pred_array2, delimiter=",")
                    
                            
                        
                                    
                    #repeat kf on prediction :   
                    if math.isnan(pt[0]):
                            print("No Human 2")
                    else:
                        b = 0
                        for k in range(7):  
                            b = b + 1                  
                            predicted2 = kf_cv.predict(predicted2[0],predicted2[1])
                            print("predicted 2:",predicted2) 
                            prediction2.header.frame_id = "camera_rgb_optical_frame"
                            prediction2.ns = "basic_shapes"
                            prediction2.id = b
                            prediction2.type = 1
                            prediction2.pose.position.x =  predicted2[0]
                            prediction2.pose.position.y = 0.0
                            prediction2.pose.position.z = predicted2[1]
                            prediction2.pose.orientation.x = 1.0
                            prediction2.pose.orientation.y = 1.0
                            prediction2.pose.orientation.z = 0.0
                            prediction2.pose.orientation.w = 0.0
                            prediction2.scale.x = 0.7
                            prediction2.scale.y = 0.7
                            prediction2.scale.z = 0.7
                            prediction2.color.a = 1.0
                            prediction2.color.r = 0.0
                            prediction2.color.g = 1.0
                            prediction2.color.b = 0.0
                            
                            #publish marker for predicted position:
                            self.prediction_marker2.publish(prediction2)
                            pred_array2.append(predicted2)
                            # np.savetxt("pred2.txt", pred_array2, delimiter=",")
                            
                            
                    #publish predicted positions of human:
                    # prediction_msg2.header = data.header
                    # prediction_msg2.x = predicted2[0]
                    # prediction_msg2.z = predicted2[1]
                    # self.pred_human2.publish(prediction_msg2)
                    
                            
                
            
            #***********************************************************
                    
                if labels[i] == 2 :
                    useful_cluster3.append(components[i])
                    point3 = components[i] 
                    x3.append(point3[0])
                    z3.append(point3[1])
            
                
                    meanx3 = np.nanmean(x3)
                    meanz3 = np.nanmean(z3)                    
                    
                    #get x,z position cordinates for kf:
                    pos3 = [meanx3,meanz3,0.0] 
                    # human3_array.append(pos3)
                    # np.savetxt("org3.txt", human3_array, delimiter=",")
                    
                    #add positions to array and transform:
                    if len(transform_array3)<15:
                        if len(transform_array3)==0:
                            transform_array3.append(pos3)
                            
                        else:
                            for i in range(len(transform_array3)):
                                transform_array3[i] = quaternion.rotate_vectors(q_rot,transform_array3[i]) + translation
                            transform_array3.append(pos3)
                            
                    else:
                        transform_array3.pop(0)
                        for i in range(len(transform_array1)):
                                transform_array3[i] = quaternion.rotate_vectors(q_rot,transform_array3[i]) + translation
                        transform_array3.append(pos3)
                        
                    
                    #publish current human position
                    position_msg3.header = data.header
                    position_msg3.x = meanx3 #pos3_trans[0] #meanx3
                    position_msg3.z = meanz3 #pos3_trans[1] #meanz3
                    self.pose_human3.publish(position_msg3)
                    
                    #create marker at current mean position of human:
                    Human_Marker_cube3.header.stamp = rospy.Time.now()
                    Human_Marker_cube3.header.frame_id = "camera_rgb_optical_frame"
                    Human_Marker_cube3.ns = "basic_shapes_3"
                    Human_Marker_cube3.id = 3
                    Human_Marker_cube3.type = 1
                    Human_Marker_cube3.pose.position.x = meanx3 #pos3_trans[0] #meanx3 #transformed_pose.position.x
                    Human_Marker_cube3.pose.position.y = 0.0
                    Human_Marker_cube3.pose.position.z = meanz3 #pos3_trans[1] #meanz3 #transformed_pose.position.z
                    Human_Marker_cube3.pose.orientation.x = 1.0
                    Human_Marker_cube3.pose.orientation.y = 1.0
                    Human_Marker_cube3.pose.orientation.z = 0.0
                    Human_Marker_cube3.pose.orientation.w = 0.0
                    Human_Marker_cube3.scale.x = 0.7
                    Human_Marker_cube3.scale.y = 0.7
                    Human_Marker_cube3.scale.z = 0.7
                    Human_Marker_cube3.color.a = 1.0
                    Human_Marker_cube3.color.r = 1.0
                    Human_Marker_cube3.color.g = 0.0
                    Human_Marker_cube3.color.b = 0.0
                    
                    #publish marker at current mean position of human:
                    self.human_marker3.publish(Human_Marker_cube3)
                    
                    #start kalman filtering:
                    kf_cv = KalmanFilter()
                    
                
                    for pt in transform_array3: 
                    
                        predicted3 = kf_cv.predict(pt[0], pt[1]) 
                    pred_array3.append(predicted3)
                    # np.savetxt("pred3.txt", pred_array3, delimiter=",")
                    
                            
                    if math.isnan(pt[0]):
                            print("No Human 3")
                    else:       
                        c = 0                        
                        #repeat kf on prediction :      
                        for n in range(7):  
                            c = c + 1                  
                            predicted3 = kf_cv.predict(predicted3[0],predicted3[1])
                            print("predicted 3:",predicted3) 
                            prediction3.header.frame_id = "camera_rgb_optical_frame"
                            prediction3.ns = "basic_shapes"
                            prediction3.id = c
                            prediction3.type = 1
                            prediction3.pose.position.x =  predicted3[0]
                            prediction3.pose.position.y = 0.0
                            prediction3.pose.position.z = predicted3[1]
                            prediction3.pose.orientation.x = 1.0
                            prediction3.pose.orientation.y = 1.0
                            prediction3.pose.orientation.z = 0.0
                            prediction3.pose.orientation.w = 0.0
                            prediction3.scale.x = 0.7
                            prediction3.scale.y = 0.7
                            prediction3.scale.z = 0.7
                            prediction3.color.a = 1.0
                            prediction3.color.r = 0.0
                            prediction3.color.g = 1.0
                            prediction3.color.b = 0.0
                            
                            #publish marker for predicted position:                        
                            self.prediction_marker3.publish(prediction3)
                            pred_array3.append(predicted3)
                            # np.savetxt("pred3.txt", pred_array3, delimiter=",")
                            
                            
                        # #publish predicted positions of human:
                        # prediction_msg3.header = data.header
                        # prediction_msg3.x = predicted3[0]
                        # prediction_msg3.z = predicted3[1]
                        # self.pred_human3.publish(prediction_msg3)
                
            #*****************************************************************************
            
                    
            
                
   
                print()  
        
        end = timer()
        print(timedelta(seconds=end-start)) #in seconds
#****************************************************************************************************************

     
def main():
        
        
        rospy.init_node('CLUSTERING_PREDICTION', anonymous=False)         
        pr = Prediction()        
        while not rospy.is_shutdown():
            rospy.spin() 
            
 
     

if __name__ == '__main__':
    main()
    
 