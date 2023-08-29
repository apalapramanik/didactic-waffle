#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import torch
from tb_apala.pointnet2_cls_msg import get_model



class PointNetNode:
    def __init__(self):
        rospy.init_node('pointnet_node')
        self.model = get_model(num_class=40) 
        self.model.load_state_dict(torch.load('best_model.pth', map_location=torch.device('cpu')))
        self.model.eval()

        rospy.Subscriber('/camera/depth/points', PointCloud2, self.point_cloud_callback)

    def point_cloud_callback(self, msg):
        pc_data = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        point_cloud = np.array(list(pc_data))

        # Preprocess point cloud data if needed

        # Pass the preprocessed point cloud through the PointNet++ model
        point_cloud_tensor = torch.tensor(point_cloud, dtype=torch.float32)
        with torch.no_grad():
            predictions, _ = self.model(point_cloud_tensor.unsqueeze(0))  # Add batch dimension

        predicted_class = torch.argmax(predictions, dim=1).item()

        rospy.loginfo(f"Predicted class: {predicted_class}")

if __name__ == '__main__':
    try:
        pointnet_node = PointNetNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
