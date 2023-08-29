#!/usr/bin/env python

import rospy
# import ros_numpy
import numpy as np
import open3d as o3d
# import pcl
import torch
import struct
from sensor_msgs.msg import PointCloud2 as pc2
import rospkg
# from pointnet2_cls_msg import get_model
from tb_apala.msg import pcl_pred
# from pointnet2_cls_msg import get_model
from tb_apala.PintView_GCN import PointViewGCN

class PointCloudClassifier:
    def __init__(self, model_path, point_cloud_topic):     

        # Subscribe to the PointCloud topic
        self.point_cloud_subscriber = rospy.Subscriber(point_cloud_topic, pc2, self.callback_pointcloud)
     
        self.prediction_publisher = rospy.Publisher('/point_cloud_predictions', pcl_pred, queue_size=1)
        

    def apply_voxel_grid_filter(self, point_cloud, leaf_size):
        # Convert PointCloud2 to NumPy array
        # pcl_array = ros_numpy.point_cloud2.pointcloud2_to_xyzrgb_array(point_cloud)
        # pcl_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(point_cloud, remove_nans=False) 
        
        # pcl_array = self.pointcloud2_to_array(point_cloud)
        # print(pcl_array.shape) 
        
        # # Convert NumPy array to open3d point cloud format
        # o3d_cloud = o3d.geometry.PointCloud()
        # o3d_cloud.points = o3d.utility.Vector3dVector(pcl_array[:, :3])
        # o3d_cloud.colors = o3d.utility.Vector3dVector(pcl_array[:, 3:6] / 255.0)  # Normalize RGB values to [0, 1]

        # Create Voxel Grid filter object
        # voxel_down_pcd = o3d_cloud.voxel_down_sample(voxel_size=leaf_size)
        voxel_down_pcd = o3d.geometry.VoxelGrid.create_from_point_cloud(point_cloud,
                                                            voxel_size=0.05)
        
        # print(voxel_down_pcd.shape)
       

        # Convert back to NumPy array
        downsampled_pcl = np.hstack(np.asarray(voxel_down_pcd.points))#, np.asarray(voxel_down_pcd.colors) * 255.0))
        # print(downsampled_pcl.shape)

        return downsampled_pcl

    def callback_pointcloud(self, data):
        print("cb")
        # print(data.shape) 
            
        # Preprocess the point cloud data using Voxel Grid filter
        # leaf_size = 0.01  # Adjust the leaf size as needed (voxel size in meters)
        # downsampled_pcl = self.apply_voxel_grid_filter(data, leaf_size)  
        pcl_array = self.pointcloud2_to_array(data)
        print(pcl_array.shape)      
        # tensor_pcl = torch.tensor(downsampled_pcl, dtype=torch.float32).unsqueeze(0).permute(0, 2, 1)
        tensor_pcl = torch.tensor(pcl_array)
        # print(tensor_pcl.cuda)
        
        # self.model = get_model(num_class=40, normal_channel=True)  # Assuming you are using ModelNet40 with normals
        self.model = PointViewGCN(name='apala')
        self.model.load_state_dict(torch.load(model_path), strict = False)
        self.model.eval()

        with torch.no_grad():
            # Assuming you have the model loaded as 'model'
            predictions = self.model(tensor_pcl.cuda)
            print(predictions)


       
        
    
    def pointcloud2_to_array(self, point_cloud):
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

        return np.array(points_list, dtype=np.float32)

if __name__ == '__main__':
    rospy.init_node('point_cloud_classifier', anonymous=True)
    
    # Initialize the ROS package
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('tb_apala')
    print(package_path)

    # Specify the path to the model file relative to the package
    # model_path = package_path + '/src/best_model.pth'
    model_path = package_path + '/src/pointnet_on_single_view.pth'

    # Path to the pretrained model .pth file
    # model_path = '/scripts/pointnet_on_single_view.pth'

    # Point cloud topic to subscribe to
    point_cloud_topic = '/camera/depth/points'

    # Create the PointCloudClassifier instance
    point_cloud_classifier = PointCloudClassifier(model_path, point_cloud_topic)

    rospy.spin()

