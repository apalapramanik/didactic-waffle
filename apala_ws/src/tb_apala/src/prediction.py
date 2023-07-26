"""
    Author:  Apala Pramanik 
    Project: Autonomous Robot in Construction site
    Advised by: Dr.Dung Hoang Tran, Dr.Kyungki Kim
    
"""
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2 as pc2
from std_msgs.msg import String
import quaternion # https://github.com/moble/quaternion
from nav_msgs.msg import Odometry
import numpy as np
from tb_apala.msg import position
import struct
from visualization_msgs.msg import Marker
from marker_publisher import marker
from kf_predictors import FilterEstimator
from std_msgs.msg import Float32MultiArray
from cv2 import HuMoments
import rospy
from cmath import isnan, nan, sqrt
from os import device_encoding
from numpy import NaN, cov, poly
from sklearn.cluster import DBSCAN
# import ros_numpy




filter_type = "kf"
steps = 5

odom_pose = []
odom_or = []
rot_array = []
trans_array = []
transform_array1 = []
transform_array2 = []
transform_array3 = []

human1_array = []
human2_array = []


class predict:
    def __init__(self):
        
        #subscribers
        rospy.Subscriber("projected",pc2,self.cloud_callback,queue_size=10)  
        rospy.Subscriber("cp_flag", String, self.cp_flag_callback, queue_size=10 )
        rospy.Subscriber("odom", Odometry, self.odom_callback,queue_size=10)
        
        #publishers
        self.pose_human1 = rospy.Publisher("position_h1", position,queue_size=1)
        self.pose_human2 = rospy.Publisher("position_h2", position,queue_size=1)
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
            pcl_np = pointcloud2_to_numpy(data)
            # print(pcl_np)
            
            
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
            
            # rospy.loginfo("Predicting now")
                
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
                    pos1 = [meanx1,meanz1, 0.0] #check x, y, z order
                    pos1b = [meanx1,meanz1]
                    human1_array.append(pos1b)
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
                    predictions_array1, error1= filter_estimator1.ukf_caller()
                    rospy.loginfo("Prediction done!")
                    
                    # for evaluation:
                    # filter_estimator11 = FilterEstimator(transform_array2, steps)
                    # predictions_array122, error12= filter_estimator11.kf_caller()
                    
                    # filter_estimator12 = FilterEstimator(transform_array2, steps)
                    # predictions_array123, error13= filter_estimator12.ekf_caller()
                    
                    # filter_estimator13 = FilterEstimator(transform_array2, steps)
                    # predictions_array124, error14= filter_estimator13.ukf_caller()
                    
                    # filter_estimator14 = FilterEstimator(transform_array2, steps)
                    # predictions_array125, error15= filter_estimator14.enkf_caller()
                    
                    
                
                    b = 0
                    for pt in range(len(predictions_array1)):                        
                        point = predictions_array1[pt]
                        b = b+1
                        marker.publish_prediction_marker(b, name = "pred_human1", cord_x= point[0], cord_y=0.0, cord_z= point[1])
                        
                     
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
                    pos2b = [meanx2,meanz2] 
                    human2_array.append(pos2b)
                    np.savetxt("org2.txt", human2_array, delimiter=",")
                    
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
                    predictions_array2, error2= filter_estimator2.kf_caller()
                    
                    
                    #for evaluation:
                    # filter_estimator21 = FilterEstimator(transform_array2, steps)
                    # predictions_array21, error21= filter_estimator21.kf_caller()
                    
                    # filter_estimator22 = FilterEstimator(transform_array2, steps)
                    # predictions_array22, error22= filter_estimator22.ekf_caller()
                    
                    # filter_estimator23 = FilterEstimator(transform_array2, steps)
                    # predictions_array23, error23= filter_estimator23.ukf_caller()
                    
                    # filter_estimator24 = FilterEstimator(transform_array2, steps)
                    # predictions_array24, error24= filter_estimator24.enkf_caller()
                    
                    
                    a = 0
                    for pt in range(len(predictions_array2)):  
                        a = a+1                     
                        marker.publish_prediction_marker(a, name = "pred_human2", cord_x= predictions_array2[pt][0], cord_y=0.0, cord_z= predictions_array2[pt][1])
                    
                    pred2_distance_array = []


                    for y in predictions_array2:
                        pred2_distance = ((y[0]**2 + y[1]**2)**0.5)
                        pred2_distance_array.append(pred2_distance)
                   
                    pred2_array_msg.data = pred2_distance_array
                    self.pred2_array.publish(pred2_array_msg)   
                    
                # rospy.loginfo("Prediction done!")
            


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

def main():
        rospy.init_node('clustering_prediction_node', anonymous=False)         
        pr = predict()        
        while not rospy.is_shutdown():
            rospy.spin()      
            
if __name__ == '__main__':
    main()   
    

