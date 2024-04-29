#!/usr/bin/env python

import cv2
import numpy as np
import math
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String, Float32
from visualization_msgs.msg import Marker
from monitoring.msg import yolodepth

import torch

class ObjectDetectionNode:
    def __init__(self):
        
        # Load the YOLOv5 model
        self.model_0 = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
        self.model_1 = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
        self.model_2 = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)

        # Initialize the CvBridge
        self.bridge_0 = CvBridge()
        self.bridge_1 = CvBridge()
        self.bridge_2 = CvBridge()

        # Initialize the color image and depth image
        self.color_image_0 = None
        self.depth_image_0 = None
        
        self.color_image_1 = None
        self.depth_image_1 = None
        
        self.color_image_2 = None
        self.depth_image_2 = None

        # Subscribes to the color and depth image topics tb3_0
        rospy.Subscriber('tb3_0/camera/rgb/image_raw', Image, self.color_image_callback_tb3_0)
        rospy.Subscriber('tb3_0/camera/depth/image_raw', Image, self.depth_image_callback_tb3_0)
        
        # Subscribes to the color and depth image topics tb3_1
        rospy.Subscriber('tb3_1/camera/rgb/image_raw', Image, self.color_image_callback_tb3_1)
        rospy.Subscriber('tb3_1/camera/depth/image_raw', Image, self.depth_image_callback_tb3_1)
        
        # Subscribes to the color and depth image topics tb3_2
        rospy.Subscriber('tb3_2/camera/rgb/image_raw', Image, self.color_image_callback_tb3_2)
        rospy.Subscriber('tb3_2/camera/depth/image_raw', Image, self.depth_image_callback_tb3_2)
        

        # Publishers  tb3_0
        self.annotated_image_pub_tb3_0 = rospy.Publisher('tb3_0/annotated_image', Image, queue_size=10)
        self.yolo_info_pub_tb3_0 = rospy.Publisher('tb3_0/yolo_depth_topic', yolodepth, queue_size=10)
        self.cloud_processing_pub_tb3_0 = rospy.Publisher('tb3_0/cp_flag', String, queue_size=10)
        
         # Publishers tb3_1
        self.annotated_image_pub_tb3_1 = rospy.Publisher('tb3_1/annotated_image', Image, queue_size=10)
        self.yolo_info_pub_tb3_1 = rospy.Publisher('tb3_1/yolo_depth_topic', yolodepth, queue_size=10)
        self.cloud_processing_pub_tb3_1 = rospy.Publisher('tb3_1/cp_flag', String, queue_size=10)
        
         # Publishers tb3_2
        self.annotated_image_pub_tb3_2 = rospy.Publisher('tb3_2/annotated_image', Image, queue_size=10)
        self.yolo_info_pub_tb3_2 = rospy.Publisher('tb3_2/yolo_depth_topic', yolodepth, queue_size=10)
        self.cloud_processing_pub_tb3_2 = rospy.Publisher('tb3_2/cp_flag', String, queue_size=10)
        

        #create custom msg instance
        self.yolo_depth_msg_tb3_0 = yolodepth()
        self.yolo_depth_msg_tb3_1 = yolodepth()
        self.yolo_depth_msg_tb3_2 = yolodepth()
        
    ############################################################ tb3_0 #########################################################################################################################################
        
        
    def color_image_callback_tb3_0(self, msg):
        # Convert ROS Image to OpenCV image
        self.color_image_0 = self.bridge_0.imgmsg_to_cv2(msg, "bgr8")

        # Perform object detection if both color and depth images are available
        if self.color_image_0 is not None and self.depth_image_0 is not None:
            self.detect_objects_tb3_0()

    def depth_image_callback_tb3_0(self, msg):
        # Convert ROS Image to OpenCV image
        self.depth_image_0 = self.bridge_0.imgmsg_to_cv2(msg, desired_encoding='passthrough')    
        
    def detect_objects_tb3_0(self):
        
        # Set the depth scale
        # depth_scale = 0.0010000000474974513
        # depth_scale = 0.001

        # Convert the depth image to meters
        depth_image_0 = self.depth_image_0 #* depth_scale

        # Detect objects using YOLOv5
        results = self.model_0(self.color_image_0)
        
        # Lists to store information about detected persons and other objects
        persons = []
        other_objects = []


        # Process the results
        for result in results.xyxy[0]:
            x1, y1, w, h, confidence, class_id = result
            # print(type(confidence))

            # Calculate the center of the bounding box
            center_x = (x1 + w) / 2
            center_y = (y1 + h) / 2
            

            # Calculate the depth at the center of the bounding box
            object_depth = depth_image_0[int(center_y), int(center_x)]
            
            # Calculate the minimum depth within the region covered by the bounding box
            # object_depth = np.min(depth_image_0[int(y1):int(h), int(x1):int(w)])

            
            # Publish the class ID, class name, depth and confidence
            self.yolo_depth_msg_tb3_0.class_id = int(class_id)
            self.yolo_depth_msg_tb3_0.label = self.model_0.names[int(class_id)]
            self.yolo_depth_msg_tb3_0.depth = object_depth            
            self.yolo_depth_msg_tb3_0.confidence = confidence
            self.yolo_info_pub_tb3_0.publish(self.yolo_depth_msg_tb3_0) 
                    
            # object_depth = "{:.3e}m".format(object_depth) 
            depth = str(object_depth)      

            # Draw a rectangle around the object
            cv2.rectangle(self.color_image_0, (int(x1), int(y1)), (int(w), int(h)), (153, 26, 25), 2)

            # Add text for depth
            cv2.putText(self.color_image_0, depth, (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (153, 26, 25), 1)
            cv2.putText(self.color_image_0, self.model_0.names[int(class_id)] , (int(x1), int(y1)+10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (153, 26, 25), 1)
            
            class_id = int(class_id)
        
            # Check if the detected object is a person (class_id 0 for 'person')
            if class_id == 0:
                persons.append((center_x, center_y, object_depth))
                # print(len(persons))
            else:
                other_objects.append((center_x, center_y, object_depth))
                # print(len(other_objects))

        # If no person is detected, publish "no" and return
        if len(persons) == 0 and len(other_objects) == 0:
            self.cloud_processing_pub_tb3_0.publish("no")  
            
        elif len(persons)> 0 and len(other_objects) == 0:  
            self.cloud_processing_pub_tb3_0.publish("yes") 
            
        elif len(persons)== 0 and len(other_objects) > 0:
            self.cloud_processing_pub_tb3_0.publish("no")    
        else:
            # Find the closest person to compare distances
            closest_person = min(persons, key=lambda x: x[2])
            # print("closest person distance:", closest_person[2])
            rospy.loginfo("closest person distance: %f", closest_person[2])           
            for obj in other_objects:
                if obj[2] < closest_person[2]:
                    # print("object_distance:", obj[2], "person distance:", closest_person[2])
                    self.cloud_processing_pub_tb3_0.publish("no")
                else:
                    # print("object_distance:", obj[2], "person distance:", closest_person[2])
                    self.cloud_processing_pub_tb3_0.publish("yes") 
                

        # Convert the annotated image back to ROS Image message
        annotated_image_msg_0 = self.bridge_0.cv2_to_imgmsg(self.color_image_0, encoding='bgr8')
        annotated_image_msg_0.header.frame_id = 'tb3_0_tf/camera_rgb_optical_frame'  

        # Publish the annotated image
        self.annotated_image_pub_tb3_0.publish(annotated_image_msg_0)

        # Show the annotated image
        # cv2.imshow("Annotated Image", self.color_image_0)
        # cv2.waitKey(1)
        
        
        
    ############################################################ tb3_1 ########################################################################################################################################
    
    def color_image_callback_tb3_1(self, msg):
        # Convert ROS Image to OpenCV image
        self.color_image_1 = self.bridge_1.imgmsg_to_cv2(msg, "bgr8")

        # Perform object detection if both color and depth images are available
        if self.color_image_1 is not None and self.depth_image_1 is not None:
            self.detect_objects_tb3_1()

    def depth_image_callback_tb3_1(self, msg):
        # Convert ROS Image to OpenCV image
        self.depth_image_1 = self.bridge_1.imgmsg_to_cv2(msg, desired_encoding='passthrough')    
        
    def detect_objects_tb3_1(self):
        
        # Set the depth scale
        # depth_scale = 0.0010000000474974513
        # depth_scale = 0.001

        # Convert the depth image to meters
        depth_image_1 = self.depth_image_1 #* depth_scale

        # Detect objects using YOLOv5
        results = self.model_1(self.color_image_1)
        
        # Lists to store information about detected persons and other objects
        persons = []
        other_objects = []


        # Process the results
        for result in results.xyxy[0]:
            x1, y1, w, h, confidence, class_id = result
            # print(type(confidence))

            # Calculate the center of the bounding box
            center_x = (x1 + w) / 2
            center_y = (y1 + h) / 2
            

            # Calculate the depth at the center of the bounding box
            object_depth = depth_image_1[int(center_y), int(center_x)]
            
            # Calculate the minimum depth within the region covered by the bounding box
            # object_depth = np.min(depth_image_1[int(y1):int(h), int(x1):int(w)])

            
            # Publish the class ID, class name, depth and confidence
            self.yolo_depth_msg_tb3_1.class_id = int(class_id)
            self.yolo_depth_msg_tb3_1.label = self.model_1.names[int(class_id)]
            self.yolo_depth_msg_tb3_1.depth = object_depth            
            self.yolo_depth_msg_tb3_1.confidence = confidence
            self.yolo_info_pub_tb3_1.publish(self.yolo_depth_msg_tb3_1) 
                    
            # object_depth = "{:.3e}m".format(object_depth) 
            depth = str(object_depth)      

            # Draw a rectangle around the object
            cv2.rectangle(self.color_image_1, (int(x1), int(y1)), (int(w), int(h)), (153, 26, 25), 2)

            # Add text for depth
            cv2.putText(self.color_image_1, depth, (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (153, 26, 25), 1)
            cv2.putText(self.color_image_1, self.model_1.names[int(class_id)] , (int(x1), int(y1)+10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (153, 26, 25), 1)
            
            class_id = int(class_id)
        
            # Check if the detected object is a person (class_id 0 for 'person')
            if class_id == 0:
                persons.append((center_x, center_y, object_depth))
                # print(len(persons))
            else:
                other_objects.append((center_x, center_y, object_depth))
                # print(len(other_objects))

        # If no person is detected, publish "no" and return
        if len(persons) == 0 and len(other_objects) == 0:
            self.cloud_processing_pub_tb3_1.publish("no")  
            
        elif len(persons)> 0 and len(other_objects) == 0:  
            self.cloud_processing_pub_tb3_1.publish("yes") 
            
        elif len(persons)== 0 and len(other_objects) > 0:
            self.cloud_processing_pub_tb3_1.publish("no")    
        else:
            # Find the closest person to compare distances
            closest_person = min(persons, key=lambda x: x[2])
            # print("closest person distance:", closest_person[2])
            rospy.loginfo("closest person distance: %f", closest_person[2])           
            for obj in other_objects:
                if obj[2] < closest_person[2]:
                    # print("object_distance:", obj[2], "person distance:", closest_person[2])
                    self.cloud_processing_pub_tb3_1.publish("no")
                else:
                    # print("object_distance:", obj[2], "person distance:", closest_person[2])
                    self.cloud_processing_pub_tb3_1.publish("yes") 
                

        # Convert the annotated image back to ROS Image message
        annotated_image_msg_1 = self.bridge_1.cv2_to_imgmsg(self.color_image_1, encoding='bgr8')
        annotated_image_msg_1.header.frame_id = 'tb3_1_tf/camera_rgb_optical_frame'  

        # Publish the annotated image
        self.annotated_image_pub_tb3_1.publish(annotated_image_msg_1)

        # Show the annotated image
        # cv2.imshow("Annotated Image", self.color_image_1)
        # cv2.waitKey(1)
        
    ############################################################ tb3_2 ########################################################################################################################################
    
    def color_image_callback_tb3_2(self, msg):
        # Convert ROS Image to OpenCV image
        self.color_image_2 = self.bridge_2.imgmsg_to_cv2(msg, "bgr8")

        # Perform object detection if both color and depth images are available
        if self.color_image_2 is not None and self.depth_image_2 is not None:
            self.detect_objects_tb3_2()

    def depth_image_callback_tb3_2(self, msg):
        # Convert ROS Image to OpenCV image
        self.depth_image_2 = self.bridge_2.imgmsg_to_cv2(msg, desired_encoding='passthrough')    
        
    def detect_objects_tb3_2(self):
        
        # Set the depth scale
        # depth_scale = 0.0010000000474974513
        # depth_scale = 0.001

        # Convert the depth image to meters
        depth_image_2 = self.depth_image_2 #* depth_scale

        # Detect objects using YOLOv5
        results = self.model_2(self.color_image_2)
        
        # Lists to store information about detected persons and other objects
        persons = []
        other_objects = []


        # Process the results
        for result in results.xyxy[0]:
            x1, y1, w, h, confidence, class_id = result
            # print(type(confidence))

            # Calculate the center of the bounding box
            center_x = (x1 + w) / 2
            center_y = (y1 + h) / 2
            

            # Calculate the depth at the center of the bounding box
            object_depth = depth_image_2[int(center_y), int(center_x)]
            
            # Calculate the minimum depth within the region covered by the bounding box
            # object_depth = np.min(depth_image_2[int(y1):int(h), int(x1):int(w)])

            
            # Publish the class ID, class name, depth and confidence
            self.yolo_depth_msg_tb3_2.class_id = int(class_id)
            self.yolo_depth_msg_tb3_2.label = self.model_2.names[int(class_id)]
            self.yolo_depth_msg_tb3_2.depth = object_depth            
            self.yolo_depth_msg_tb3_2.confidence = confidence
            self.yolo_info_pub_tb3_2.publish(self.yolo_depth_msg_tb3_2) 
                    
            # object_depth = "{:.3e}m".format(object_depth) 
            depth = str(object_depth)      

            # Draw a rectangle around the object
            cv2.rectangle(self.color_image_2, (int(x1), int(y1)), (int(w), int(h)), (153, 26, 25), 2)

            # Add text for depth
            cv2.putText(self.color_image_2, depth, (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (153, 26, 25), 1)
            cv2.putText(self.color_image_2, self.model_2.names[int(class_id)] , (int(x1), int(y1)+10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (153, 26, 25), 1)
            
            class_id = int(class_id)
        
            # Check if the detected object is a person (class_id 0 for 'person')
            if class_id == 0:
                persons.append((center_x, center_y, object_depth))
                # print(len(persons))
            else:
                other_objects.append((center_x, center_y, object_depth))
                # print(len(other_objects))

        # If no person is detected, publish "no" and return
        if len(persons) == 0 and len(other_objects) == 0:
            self.cloud_processing_pub_tb3_2.publish("no")  
            
        elif len(persons)> 0 and len(other_objects) == 0:  
            self.cloud_processing_pub_tb3_2.publish("yes") 
            
        elif len(persons)== 0 and len(other_objects) > 0:
            self.cloud_processing_pub_tb3_2.publish("no")    
        else:
            # Find the closest person to compare distances
            closest_person = min(persons, key=lambda x: x[2])
            # print("closest person distance:", closest_person[2])
            rospy.loginfo("closest person distance: %f", closest_person[2])           
            for obj in other_objects:
                if obj[2] < closest_person[2]:
                    # print("object_distance:", obj[2], "person distance:", closest_person[2])
                    self.cloud_processing_pub_tb3_2.publish("no")
                else:
                    # print("object_distance:", obj[2], "person distance:", closest_person[2])
                    self.cloud_processing_pub_tb3_2.publish("yes") 
                

        # Convert the annotated image back to ROS Image message
        annotated_image_msg_2 = self.bridge_2.cv2_to_imgmsg(self.color_image_2, encoding='bgr8')
        annotated_image_msg_2.header.frame_id = 'tb3_2_tf/camera_rgb_optical_frame'  

        # Publish the annotated image
        self.annotated_image_pub_tb3_2.publish(annotated_image_msg_2)

        # Show the annotated image
        # cv2.imshow("Annotated Image", self.color_image_2)
        # cv2.waitKey(1)
        
#______________________________________________________________________________________________________________________________________________________________________________________________
 

def main():
    rospy.init_node('Depth_calculation', anonymous=False)    
    obj = ObjectDetectionNode()    
    rospy.spin()

if __name__ == '__main__':
    main()
