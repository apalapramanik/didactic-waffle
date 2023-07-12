import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import torch

# Load the YOLOv5 model
model = torch.hub.load('ultralytics/yolov5', 'yolov5m', pretrained=True)

# Initialize the ROS node
rospy.init_node('object_detection_node')

# Initialize the CvBridge
bridge = CvBridge()

# Publisher for the annotated imageimport cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String, Float32

import torch

# Load the YOLOv5 model
model = torch.hub.load('ultralytics/yolov5', 'yolov5m', pretrained=True)

# Initialize the ROS node
rospy.init_node('object_detection_node')

# Initialize the CvBridge
bridge = CvBridge()

# Publisher for the annotated image
annotated_image_pub = rospy.Publisher('annotated', Image, queue_size=10)

# Publisher for the distance
distance_pub = rospy.Publisher('distance_topic', Float32, queue_size=10)

# Publisher for the depth image
depth_pub = rospy.Publisher('depth_topic', Image, queue_size=10)

# Callback function for processing the camera image
def image_callback(msg):
    # Convert ROS Image to OpenCV image
    color_image = bridge.imgmsg_to_cv2(msg, "bgr8")

    # Set the depth scale (if needed)
    depth_scale = 0.0010000000474974513

    # Convert the color image to grayscale (if needed)
    gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

    # Load the depth image (assuming it's published on a separate topic)
    depth_image_msg = rospy.wait_for_message('/camera/depth/image_raw', Image)
    depth_image = bridge.imgmsg_to_cv2(depth_image_msg, desired_encoding='passthrough')

    # Convert the depth image to meters (if needed)
    depth_image = depth_image * depth_scale

    # Detect objects using YOLOv5
    results = model(color_image)

    # Process the results
    for result in results.xyxy[0]:
        x1, y1, x2, y2, confidence, class_id = result

        # Calculate the distance to the object
        object_depth = np.median(depth_image[int(y1):int(y2), int(x1):int(x2)])
        label = f"{object_depth:.2f}m"
        distance_pub.publish(object_depth)
        depth_pub.publish(bridge.cv2_to_imgmsg(depth_image, encoding='passthrough'))

        # Draw a rectangle around the object
        cv2.rectangle(color_image, (int(x1), int(y1)), (int(x2), int(y2)), (252, 119, 30), 2)

        # Draw the bounding box
        cv2.putText(color_image, label, (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (252, 119, 30), 2)

        # Print the object's class and distance
        # rospy.loginfo(f"{model.names[int(class_id)]}: Object detected")
        rospy.loginfo(f"Class ID: {model.names[int(class_id)]},  Distance: {object_depth:.2f}m")

    # Convert the annotated image back to ROS Image message
    annotated_image_msg = bridge.cv2_to_imgmsg(color_image, encoding='bgr8')

    # Publish the annotated image
    annotated_image_pub.publish(annotated_image_msg)

# Subscribe to the camera image topic
rospy.Subscriber('/camera/rgb/image_raw', Image, image_callback)

# Spin ROS
rospy.spin()

annotated_image_pub = rospy.Publisher('annotated', Image, queue_size=10)

# Callback function for processing the camera image
def image_callback(msg):
    # Convert ROS Image to OpenCV image
    color_image = bridge.imgmsg_to_cv2(msg, "bgr8")

    # Set the depth scale (if needed)
    depth_scale = 0.0010000000474974513

    # Convert the color image to grayscale (if needed)
    gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

    # Load the depth image (assuming it's published on a separate topic)
    depth_image_msg = rospy.wait_for_message('/camera/depth/image_raw', Image)
    depth_image = bridge.imgmsg_to_cv2(depth_image_msg, desired_encoding='passthrough')

    # Convert the depth image to meters (if needed)
    depth_image = depth_image * depth_scale

    # Detect objects using YOLOv5
    results = model(color_image)

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

        # Print the object's class and distance
        rospy.loginfo(f"{model.names[int(class_id)]}: Object detected")

    # Convert the annotated image back to ROS Image message
    annotated_image_msg = bridge.cv2_to_imgmsg(color_image, encoding='bgr8')

    # Publish the annotated image
    annotated_image_pub.publish(annotated_image_msg)

# Subscribe to the camera image topic
rospy.Subscriber('/camera/rgb/image_raw', Image, image_callback)

# Spin ROS
rospy.spin()
