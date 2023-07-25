"""
    Author:  Apala Pramanik 
    Project: Autonomous Robot in Construction site
    Advised by: Dr.Dung Hoang Tran, Dr.Kyungki Kim
    
    
"""


import numpy as np
import math
import rospy
from sensor_msgs.msg import PointCloud2 as pc2
from std_msgs.msg import String
import quaternion # https://github.com/moble/quaternion
from nav_msgs.msg import Odometry
import cv2
import numpy as np
from tb_apala.msg import position
from filterpy.monte_carlo import systematic_resample
from filterpy.kalman import ExtendedKalmanFilter
from filterpy.kalman import UnscentedKalmanFilter
from filterpy.kalman import MerweScaledSigmaPoints
from filterpy.kalman import EnsembleKalmanFilter
import pandas as pd
import struct
from visualization_msgs.msg import Marker
from marker_publisher import marker
from eval_pred import FilterEstimator
from std_msgs.msg import Float32MultiArray
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



filter_type = "kf"
steps = 5

odom_pose = []
odom_or = []
rot_array = []
trans_array = []
transform_array1 = []
transform_array2 = []
transform_array3 = []


class predict:
    def __init__(self):
        
        #subscribers
        rospy.Subscriber("projected",pc2,self.cloud_callback,queue_size=10)  
        rospy.Subscriber("cp_flag", String, self.cp_flag_callback, queue_size=10 )
        rospy.Subscriber("odom", Odometry, self.odom_callback,queue_size=10)
        
        self.pose_human1 = rospy.Publisher("position_h1", position,queue_size=1)
        self.pose_human2 = rospy.Publisher("position_h2", position,queue_size=1)
        
        # self.pred_human1 = rospy.Publisher("prediction_h1", position,queue_size=1)
        # self.pred_human2 = rospy.Publisher("prediction_h2", position,queue_size=1)
        
        self.pred1_array = rospy.Publisher("pred1_array",Float32MultiArray,queue_size=10)
        self.pred2_array = rospy.Publisher("pred2_array",Float32MultiArray,queue_size=10)
        
        self.flag = "no"
        
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
        
        position_msg1 = position()
        pred1_array_msg = Float32MultiArray()   
        position_msg2 = position()
        pred2_array_msg = Float32MultiArray()
 
          
            
        if self.flag == 'yes':
            
            #convert point cloud to numpy array:
            pcl_np = pointcloud2_to_array(data)
            
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
                
            xz_np_array = np.array(xzarray)
    
            
            ''' 
            compute DBSCAN - change eps and min_samples as required,
            eps: min distance between points
            learn more from - 
            https://scikit-learn.org/stable/modules/generated/sklearn.cluster.DBSCAN.html 
            
            '''
            
            #start db scan:
            DBSCAN_cluster = DBSCAN(eps=0.5, min_samples=30).fit(xz_np_array) #0.5, 30
            labels = DBSCAN_cluster.labels_
            components = DBSCAN_cluster.components_ #copy of each core sample found by training
            # feature = DBSCAN_cluster.n_features_in_ #number of features seen during fit
            rospy.loginfo("Clustered point cloud")
            
            
             
            useful_cluster1 = []
            useful_cluster2 = []
            # useful_cluster3 = []
            x1 = []
            z1 = []
            x2 = []
            z2 = []
            # x3 = []
            # z3 = []
            
            rospy.loginfo("Predicting now")
                
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
                    pos1 = [meanx1,meanz1,0.0] #check x, y, z order
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
                    position_msg1.x = meanx1 
                    position_msg1.z = meanz1 
                    self.pose_human1.publish(position_msg1)
                    
                    marker.publish_human_marker(name = "human1", cord_x = meanx1, cord_y = 0.0, cord_z = meanz1)
                    
                    filter_estimator1 = FilterEstimator(transform_array1, steps)
                    predictions_array1, error1= filter_estimator1.kf_caller()
                    
                
                    
                    for pt in range(len(predictions_array1)):
                        point = predictions_array1[pt]
                        marker.publish_prediction_marker(name = "pred_human2", cord_x= point[0], cord_y=0.0, cord_z= point[1])
                        
                     
                    pred1_distance_array = []


                    for x in predictions_array1:
                        pred1_distance = ((x[0]**2 + x[1]**2)**0.5)
                        pred1_distance_array.append(pred1_distance)
                   
                    pred1_array_msg.data = pred1_distance_array
                    self.pred1_array.publish(pred1_array_msg)   
                            
         
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
                    position_msg2.x = meanx2 
                    position_msg2.z = meanz2 
                    self.pose_human2.publish(position_msg2)
                    
                    marker.publish_human_marker(name = "human2", cord_x = meanx1, cord_y = 0.0, cord_z = meanz1)
                     
                    filter_estimator2 = FilterEstimator(transform_array2, steps)
                    # filter_estimator.pred(filter_type)
                    predictions_array2, error2= filter_estimator2.kf_caller()
                    
                    for pt in range(len(predictions_array2)):
                        marker.publish_prediction_marker(name = "pred_human2", cord_x= predictions_array2[pt][0], cord_y=0.0, cord_z= predictions_array2[pt][1])
                    
                    pred2_distance_array = []


                    for y in predictions_array2:
                        pred2_distance = ((y[0]**2 + y[1]**2)**0.5)
                        pred2_distance_array.append(pred2_distance)
                   
                    pred2_array_msg.data = pred2_distance_array
                    self.pred2_array.publish(pred2_array_msg)   
                    
                rospy.loginfo("Prediction done!")
            

    
def pointcloud2_to_array( point_cloud):
    
    points_list = []

    # Get the fields and their offsets from the PointCloud2 message
    fields = point_cloud.fields
    point_step = point_cloud.point_step

    # Iterate over the points in the PointCloud2 data
    for i in range(0, len(point_cloud.data), point_step):
        point = point_cloud.data[i:i + point_step]

        # Unpack the data based on field types and offsets
        x = struct.unpack_from('f', point, fields[0].offset)[0]
        y = struct.unpack_from('f', point, fields[1].offset)[0]
        z = struct.unpack_from('f', point, fields[2].offset)[0]

        # Assuming RGB fields exist (fields[3], fields[4], and fields[5])
        if len(fields) >= 6:
            r = struct.unpack_from('B', point, fields[3].offset)[0]
            g = struct.unpack_from('B', point, fields[4].offset)[0]
            b = struct.unpack_from('B', point, fields[5].offset)[0]
        else:
            # If RGB fields are not available, use default values
            r, g, b = 255, 255, 255

        # points_list.append([x, y, z, r, g, b])
        points_list.append([x, y, z])
        # rospy.loginfo("Converting point cloud to list")

    return np.array(points_list, dtype=np.float32)


def main():
        rospy.init_node('clustering_prediction_node', anonymous=False)         
        pr = predict()        
        while not rospy.is_shutdown():
            rospy.spin()      
            
if __name__ == '__main__':
    main()   
    

    def __init__(self, prediction_points, steps):
        self.prediction_points = prediction_points
        self.steps = steps
        self.predictions_array = []

    def kf_caller(self):
        kf_estimator = KalmanFilterEstimator()
        for x, y in self.prediction_points:
            predicted_x, predicted_y = kf_estimator.predict_correct(x, y)
            # print(f"Initial Point (x, y) = ({x}, {y})")
            # print(f"KF Predicted Point:", predicted_x, predicted_y)
        error = self.euclidean_distance(x, y, predicted_x, predicted_y)
        # print("error:", error)
        for i in range(steps):
            predicted_x, predicted_y = kf_estimator.predict_correct(predicted_x, predicted_y)
            self.predictions_array.append(predicted_x, predicted_y)
        #     print(f"Point {i + 1}: (x, y) = ({predicted_x}, {predicted_y})")
        # print("\n")
        return self.predictions_array, error
 
    def ekf_caller(self):  
    
        # Create ExtendedKalmanFilterEstimator instance
        ekf_estimator = ExtendedKalmanFilterEstimator()

        # Loop for each point in the input array
        for x, y in self.prediction_points:
            # print(f"Point: ({x}, {y})")

            # Extended Kalman Filter
            ekf_predicted_x, ekf_predicted_y = ekf_estimator.predict_correct(x, y)
            # print(f"EKF Predicted state after correction: (x, y) = ({ekf_predicted_x}, {ekf_predicted_y})")
        error = self.euclidean_distance(x, y, ekf_predicted_x, ekf_predicted_y)
        # print("error:", error)
            # Generate 5 more predictions using the Extended Kalman Filter
        for _ in range(steps):
            ekf_predicted_x, ekf_predicted_y = ekf_estimator.predict_correct(ekf_predicted_x, ekf_predicted_y)
            self.predictions_array.append(ekf_predicted_x, ekf_predicted_y)
            # print(f"Additional EKF Prediction: (x, y) = ({ekf_predicted_x}, {ekf_predicted_y})")
        return self.predictions_array, error
        
    def ukf_caller(self):
    
        
        # Create UnscentedKalmanFilterEstimator instance
        ukf_estimator = UnscentedKalmanFilterEstimator()

        # Initialize UKF with sigma points
        sigma = 0.1
        points = MerweScaledSigmaPoints(n=4, alpha=sigma, beta=2, kappa=0)

        # Define process noise covariance (Q) and measurement noise covariance (R)
        Q = np.diag([0.01, 0.01, 0.01, 0.01])  # Process noise covariance
        R = np.diag([0.1, 0.1])  # Measurement noise covariance

        # Initialize UKF
        ukf_estimator.ukf = UnscentedKalmanFilter(dim_x=4, dim_z=2, dt=1.0,
                                                hx=ukf_estimator.measurement_function,
                                                fx=ukf_estimator.state_transition_function,
                                                points=points)
        ukf_estimator.ukf.x = np.array([self.prediction_points[0][0], self.prediction_points[0][1], 0, 0])  # Initial state [x, y, vx, vy]
        ukf_estimator.ukf.Q = Q
        ukf_estimator.ukf.R = R

        # Loop for each point in the input array
        for x, y in self.prediction_points:
            # print(f"Point: ({x}, {y})")
            #Unscented Kalman Filter
            ukf_predicted_x, ukf_predicted_y = ukf_estimator.predict_correct(x, y)
            # print(f"UKF Predicted state after correction: (x, y) = ({ukf_predicted_x}, {ukf_predicted_y})")
        error = self.euclidean_distance(x, y, ukf_predicted_x, ukf_predicted_y)
        # print("error:", error)
        # Generate 5 more predictions using the Unscented Kalman Filter
        for _ in range(steps):
            ukf_predicted_x, ukf_predicted_y = ukf_estimator.predict_correct(ukf_predicted_x, ukf_predicted_y)
            self.predictions_array.append( ukf_predicted_x, ukf_predicted_y)
            # print(f"Additional UKF Prediction: (x, y) = ({ukf_predicted_x}, {ukf_predicted_y})")
        return self.predictions_array, error
    
    
    def enkf_caller(self):
        # Create EnsembleKalmanFilterEstimator instance
        enkf_estimator = EnsembleKalmanFilterEstimator()

        # Initialize EnKF
        state_size = 4  # State size [x, y, vx, vy]
        measurement_size = 2  # Measurement size (X and Y coordinates)
        ensemble_size = 200  # Number of ensemble members

        # Set the initial state and covariance
        initial_state = np.array([self.prediction_points[0][0], self.prediction_points[0][1], 0, 0])
        initial_covariance = np.eye(state_size) * 0.1

        enkf_estimator.enkf = EnsembleKalmanFilter(x=initial_state, P=initial_covariance, dim_z=measurement_size,
                                                dt=1.0, N=ensemble_size, hx=enkf_estimator.measurement_function,
                                                fx=enkf_estimator.state_transition_function)

        # Define process noise covariance (Q) and measurement noise covariance (R)
        enkf_estimator.enkf.Q = np.diag([0.01, 0.01, 0.01, 0.01])  # Process noise covariance
        enkf_estimator.enkf.R = np.diag([0.1, 0.1])  # Measurement noise covariance

        # Loop for each point in the input array
        for x, y in self.prediction_points:
            # print(f"Point: ({x}, {y})")
            # Ensemble Kalman Filter
            enkf_predicted_x, enkf_predicted_y = enkf_estimator.predict_correct(x, y)
            # print(f"EnKF Predicted state after correction: (x, y) = ({enkf_predicted_x}, {enkf_predicted_y})")
        error = self.euclidean_distance(x, y, enkf_predicted_x, enkf_predicted_y)
        # print("error:", error)
        # Generate 5 more predictions using the Ensemble Kalman Filter
        for _ in range(steps):
            enkf_predicted_x, enkf_predicted_y = enkf_estimator.predict_correct(enkf_predicted_x, enkf_predicted_y)
            self.predictions_array.append( enkf_predicted_x, enkf_predicted_y)
            # print(f"Additional EnKF Prediction: (x, y) = ({enkf_predicted_x}, {enkf_predicted_y})")
        return self.predictions_array, error
        
    def euclidean_distance(self, x1, y1, x2, y2):
        return np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
       
    def pred(self, filter_type):
        if filter_type == "kf":
            print("Kalman Filter:")
            self.kf_caller()

        elif filter_type == "ekf":
            print("\nExtended Kalman Filter:")
            self.ekf_caller()

        elif filter_type == "ukf":
            print("\nUnscented Kalman Filter:")
            self.ukf_caller()

        elif filter_type == "enkf":
            print("\nEnsemble Kalman Filter:")
            self.enkf_caller()

        else:
            print("Invalid filter type!")