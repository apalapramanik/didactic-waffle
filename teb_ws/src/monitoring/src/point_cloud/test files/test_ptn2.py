import torch

# model = torch.load('best_model.pth')
# print(model)


import numpy as np
from tb_apala.pointnet2_sem_seg import get_model

# Assuming you have a model architecture defined in model.py
# from model import YourPointCloudModel
from tb_apala.PintView_GCN import PointViewGCN

# Load the pre-trained model
model = get_model(num_classes=40)

checkpoint = model.load_state_dict(torch.load('best_model.pth'), strict = False)
model.eval()  # Set the model to evaluation mode

# Load and preprocess the test point cloud from the .xyz file
def load_xyz_file(file_path):
    with open(file_path, 'r') as f:
        lines = f.readlines()
    points = []
    for line in lines:
        coords = line.strip().split()
        if len(coords) == 3:  # Assuming x, y, z coordinates in each line
            points.append([float(coords[0]), float(coords[1]), float(coords[2])])
    return np.array(points)

test_xyz_file_path = 'person_0001.txt'
test_point_cloud = load_xyz_file(test_xyz_file_path)
test_point_cloud = torch.tensor(test_point_cloud, dtype=torch.float32)

# # Add a batch dimension to the point cloud
test_point_cloud = test_point_cloud.unsqueeze(0)  # Add batch dimension



# Make predictions using the model
with torch.no_grad():
    output = model(test_point_cloud)
    
