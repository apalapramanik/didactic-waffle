import torch

# model = torch.load('pointnet_on_single_view.pth')
# print(model)


import numpy as np

# Assuming you have a model architecture defined in model.py
# from model import YourPointCloudModel
from tb_apala.PintView_GCN import PointViewGCN

# Load the pre-trained model
model = PointViewGCN(name = 'Apala')
model.load_state_dict(torch.load('pointnet_on_single_view.pth'), strict = False)
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

test_xyz_file_path = 'person_0089_001.xyz'
test_point_cloud = load_xyz_file(test_xyz_file_path)
test_point_cloud = torch.tensor(test_point_cloud, dtype=torch.float32)

# Make predictions using the model
with torch.no_grad():
    output = model(test_point_cloud)
    
print(output)

# You can now use the 'output' tensor for post-processing or analysis
# For example, if it's a classification model, you might apply softmax to get probabilities
# Or if it's a regression model, it might directly give you the regression values

# Remember to adapt the post-processing based on your specific use case and model output.

# Example reshaping operation
# import torch

# x = torch.randn(1023, 3)  # Example input tensor
# views = 3
# y = x.view((int(x.shape[0] / views), views, -1))  # Reshaping

# print("Original shape:", x.shape)
# print("Reshaped shape:", y.shape)