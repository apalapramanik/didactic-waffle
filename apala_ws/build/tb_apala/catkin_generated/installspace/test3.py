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
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5m', pretrained=True)
        
        # Initialize the CvBridge
        self.bridge = CvBridge()

        # Publisher for the annotated image
        self.annotated_image_pub = rospy.Publisher('annotated_image', Image, queue_size=10)

        # Publisher for the class ID
        self.class_id_pub = rospy.Publisher('class_id_topic', String, queue_size=10)

        # Publisher for the class name
        self.class_name_pub = rospy.Publisher('class_name_topic', String, queue_size=10)

        # Publisher for the distance
        self.distance_pub = rospy.Publisher('distance_topic', Float32, queue_size=10)

        # Publisher for the depth
        self.depth_pub = rospy.Publisher('depth_topic', Image, queue_size=10)

        # Subscribe to the camera image topic
        rospy.Subscriber('/camera/rgb_image_raw', Image, self.image_callback)



    def image_callback(self, msg):
        # Convert ROS Image to OpenCV image
        color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Set the depth scale (if needed)
        depth_scale = 0.0010000000474974513

        # Convert the color image to grayscale (if needed)
        gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        # Load the depth image (assuming it's published on a separate topic)
        depth_image_msg = rospy.wait_for_message('/camera/depth/image_raw', Image)
        depth_image = self.bridge.imgmsg_to_cv2(depth_image_msg, desired_encoding='passthrough')

        # Convert the depth image to meters (if needed)
        depth_image = depth_image * depth_scale

        # Detect objects using YOLOv5
        results = self.model(color_image)

        # Process the results
        for result in results.xyxy[0]:
            x1, y1, x2, y2, confidence, class_id = result

            # Calculate the distance to the object
            object_depth = np.median(depth_image[int(y1):int(y2), int(x1):int(x2)])
            label = f"{object_depth:.2f}m"

            # Draw a rectangle around the object
            cv2.rectangle(color_image, (int(x1), int(y1)), (int(x2), int(y2)), (252, 119, 30), 2)

            # Draw the bounding box
            cv2.putText(color_image, label, (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (252, 119, 30), 2)

            # Publish the class ID, class name, distance, and depth
            self.class_id_pub.publish(str(class_id))
            self.class_name_pub.publish(self.model.names[int(class_id)])
            self.distance_pub.publish(object_depth)
            self.depth_pub.publish(self.bridge.cv2_to_imgmsg(depth_image, encoding='passthrough'))

        # Convert the annotated image back to ROS Image message
        annotated_image_msg = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')

        # Publish the annotated image
        self.annotated_image_pub.publish(annotated_image_msg)

        # Show the annotated image
        # cv2.imshow("Annotated Image", color_image)
        # cv2.waitKey(1)

      
def main(): 
     
    rospy.init_node('Depth_calculation', anonymous=False)    
    obj = ObjectDetectionNode()    
    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    main() 