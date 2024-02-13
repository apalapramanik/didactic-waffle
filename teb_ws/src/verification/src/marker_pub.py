#!/usr/bin/env python
"""
    Author:  Apala Pramanik 
    Project: Autonomous Robot Safety Verification in Construction site
    Advised by: Dr.Dung Hoang Tran    
    
"""

from visualization_msgs.msg import Marker
import rospy



human_length = 1.79 #avg length (full arm) for men
human_width = 1.79 #avg width (full arm) for men
human_height = 1.740 #avg height for men

robot_width = 0.281
robot_length = 0.306
robot_height = 0.141

class marker:
 
    def publish_pose_marker(name, cord_x, cord_y, cord_z, std_x, std_y, std_z):
        
        human_marker = rospy.Publisher(name, Marker, queue_size=0)
        prediction_marker_cube = Marker()
    
        
        prediction_marker_cube.header.stamp = rospy.Time.now()
        prediction_marker_cube.header.frame_id = "camera_rgb_optical_frame"
        prediction_marker_cube.ns = "basic_shapes_1"
        prediction_marker_cube.id = 1
        prediction_marker_cube.type = 1
        prediction_marker_cube.pose.position.x = cord_x 
        prediction_marker_cube.pose.position.y = cord_y
        prediction_marker_cube.pose.position.z = cord_z 
        prediction_marker_cube.pose.orientation.x = 1.0
        prediction_marker_cube.pose.orientation.y =  1.0
        prediction_marker_cube.pose.orientation.z = 0.0
        prediction_marker_cube.pose.orientation.w = 0.0
        prediction_marker_cube.scale.x = std_x
        prediction_marker_cube.scale.y = std_y
        prediction_marker_cube.scale.z = std_z
        prediction_marker_cube.color.a = 1.0
        prediction_marker_cube.color.r = 1.0
        prediction_marker_cube.color.g = 0.0
        prediction_marker_cube.color.b = 0.0
        
        #publish marker at current mean position of human:
        human_marker.publish(prediction_marker_cube)
        
    def publish_prediction_marker(a, name, cord_x, cord_y, cord_z, std_x, std_y, std_z):
       
        prediction_marker = rospy.Publisher(name, Marker, queue_size=0)
        pred_marker_cube = Marker()
        
        pred_marker_cube.header.stamp = rospy.Time.now()
        pred_marker_cube.header.frame_id = "camera_rgb_optical_frame"
        pred_marker_cube.ns = "basic_shapes_1"
        pred_marker_cube.id = a
        pred_marker_cube.type = 1
        pred_marker_cube.pose.position.x = cord_x 
        pred_marker_cube.pose.position.y = cord_y
        pred_marker_cube.pose.position.z = cord_z 
        pred_marker_cube.pose.orientation.x = 1.0
        pred_marker_cube.pose.orientation.y =  1.0
        pred_marker_cube.pose.orientation.z = 0.0
        pred_marker_cube.pose.orientation.w = 0.0
        pred_marker_cube.scale.x = std_x
        pred_marker_cube.scale.y = std_y
        pred_marker_cube.scale.z = std_z
        pred_marker_cube.color.a = 1.0
        pred_marker_cube.color.r = 0.0
        pred_marker_cube.color.g = 1.0
        pred_marker_cube.color.b = 0.0
        
        #publish marker at predicted positions of human:
        prediction_marker.publish( pred_marker_cube)
        
    