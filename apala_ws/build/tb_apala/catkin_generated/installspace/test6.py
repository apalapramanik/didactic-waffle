import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String, Float32

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

        # Subscribe to the color image topic
        rospy.Subscriber('/camera/rgb/image_raw', Image, self.color_image_callback)

        # Subscribe to the depth image topic
        rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_image_callback)

        # Publisher for the annotated image
        self.annotated_image_pub = rospy.Publisher('annotated_image', Image, queue_size=10)

        # Publisher for the class ID
        self.class_id_pub = rospy.Publisher('class_id_topic', String, queue_size=10)

        # Publisher for the class name
        self.class_name_pub = rospy.Publisher('class_name_topic', String, queue_size=10)

        # Publisher for the distance
        self.distance_pub = rospy.Publisher('distance_topic', Float32, queue_size=10)

        # Publisher for the depth
        self.depth_pub = rospy.Publisher('depth_topic', Float32, queue_size=10)

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
        # Set the depth scale (if needed)
        depth_scale = 0.0010000000474974513

        # Convert the depth image to meters (if needed)
        depth_image = self.depth_image * depth_scale

        # Detect objects using YOLOv5
        results = self.model(self.color_image)

        # Process the results
        for result in results.xyxy[0]:
            x1, y1, x2, y2, confidence, class_id = result

            # Calculate the center of the bounding box
            center_x = (x1 + x2) / 2
            center_y = (y1 + y2) / 2
            print(center_x, center_y)

            # Calculate the depth at the center of the bounding box
            object_depth = depth_image[int(center_y), int(center_x)]

            # Convert the depth from the depth scale to meters (if needed)
            object_depth = object_depth * depth_scale
            print(object_depth)

            label = f"{object_depth:.2f}m"

            # Draw a rectangle around the object
            cv2.rectangle(self.color_image, (int(x1), int(y1)), (int(x2), int(y2)), (252, 119, 30), 2)

            # Draw the bounding box
            cv2.putText(self.color_image, label, (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (252, 119, 30), 2)

            # Publish the class ID, class name, distance, and depth
            self.class_id_pub.publish(str(class_id))
            self.class_name_pub.publish(self.model.names[int(class_id)])
            self.distance_pub.publish(object_depth)
            self.depth_pub.publish(object_depth)

        # Convert the annotated image back to ROS Image message
        annotated_image_msg = self.bridge.cv2_to_imgmsg(self.color_image, encoding='bgr8')

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
