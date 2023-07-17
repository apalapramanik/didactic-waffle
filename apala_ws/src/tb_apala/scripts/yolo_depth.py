import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String, Float32
from visualization_msgs.msg import Marker
from tb_apala.msg import yolodepth

import torch

class ObjectDetectionNode:
    def __init__(self):
        
        # Load the YOLOv5 model
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)

        # Initialize the CvBridge
        self.bridge = CvBridge()

        # Initialize the color image and depth image
        self.color_image = None
        self.depth_image = None

        # Subscribes to the color and depth image topics
        rospy.Subscriber('/camera/rgb/image_raw', Image, self.color_image_callback)
        rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_image_callback)

        # Publishers 
        self.annotated_image_pub = rospy.Publisher('annotated_image', Image, queue_size=10)
        self.yolo_info_pub = rospy.Publisher('yolo_depth_topic', yolodepth, queue_size=10)

        #create custom msg instance
        self.yolo_depth_msg = yolodepth()
        
    def color_image_callback(self, msg):
        # Convert ROS Image to OpenCV image
        self.color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Perform object detection if both color and depth images are available
        if self.color_image is not None and self.depth_image is not None:
            self.detect_objects()

    def depth_image_callback(self, msg):
        # Convert ROS Image to OpenCV image
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def detect_objects(self):
        # Set the depth scale
        # depth_scale = 0.0010000000474974513
        depth_scale = 0.001

        # Convert the depth image to meters
        depth_image = self.depth_image * depth_scale

        # Detect objects using YOLOv5
        results = self.model(self.color_image)

        # Process the results
        for result in results.xyxy[0]:
            x1, y1, x2, y2, confidence, class_id = result
            # print(type(confidence))

            # Calculate the center of the bounding box
            center_x = (x1 + x2) / 2
            center_y = (y1 + y2) / 2
            

            # Calculate the depth at the center of the bounding box
            object_depth = depth_image[int(center_y), int(center_x)]
            
            # Calculate the minimum depth within the region covered by the bounding box
            # object_depth = np.min(depth_image[int(y1):int(y2), int(x1):int(x2)])

            # Convert the depth from the depth scale to meters  
            object_depth = object_depth.item() * depth_scale  
            
            
            # Publish the class ID, class name, depth and confidence
            self.yolo_depth_msg.label = self.model.names[int(class_id)]
            self.yolo_depth_msg.depth = object_depth            
            self.yolo_depth_msg.confidence = confidence
            self.yolo_info_pub.publish(self.yolo_depth_msg) 
                     
            object_depth = "{:.3e}m".format(object_depth) 
            depth = str(object_depth)      

            # Draw a rectangle around the object
            cv2.rectangle(self.color_image, (int(x1), int(y1)), (int(x2), int(y2)), (153, 26, 25), 2)

            # Add text for depth
            cv2.putText(self.color_image, depth, (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (153, 26, 25), 1)
            cv2.putText(self.color_image, self.model.names[int(class_id)] , (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (153, 26, 25), 1)

            
            

        # Convert the annotated image back to ROS Image message
        annotated_image_msg = self.bridge.cv2_to_imgmsg(self.color_image, encoding='bgr8')
        annotated_image_msg.header.frame_id = 'camera_rgb_optical_frame'  

        # Publish the annotated image
        self.annotated_image_pub.publish(annotated_image_msg)

        # Show the annotated image
        cv2.imshow("Annotated Image", self.color_image)
        cv2.waitKey(1)
        
        
       

def main():
    rospy.init_node('Depth_calculation', anonymous=False)    
    obj = ObjectDetectionNode()    
    rospy.spin()

if __name__ == '__main__':
    main()