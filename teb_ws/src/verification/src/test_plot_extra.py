# import re
# import matplotlib.pyplot as plt
# import numpy as np

# def plot_from_files(filename1, filename2):
#     # Load data from file
#     with open(filename1, 'r') as file:
#         data1 = file.readlines()
        
#     with open(filename2, 'r') as file:
#         data2 = file.readlines()
    

#     # Extract x and y values from each array
#     x1_values = []
#     y1_values = []
    
#     x2_values = []
#     y2_values = []
    
    

#     for line1 in data1:
#         match1 = re.findall(r"[-+]?\d*\.\d+", line1)
#         if len(match1) >= 2:
#             x1_values.append(float(match1[0]))
#             y1_values.append(float(match1[1]))
            
#     for line2 in data2:
#         match2 = re.findall(r"[-+]?\d*\.\d+", line2)
#         if len(match2) >= 2:
#             x2_values.append(float(match2[0]))
#             y2_values.append(float(match2[1]))

#     # Plotting
#     plt.figure(figsize=(10, 6))
#     plt.scatter(x1_values, y1_values, color='b', label='originals Data Points')
#     plt.scatter(x2_values, y2_values, color='r', label='calculated Data Points')
    
#     # plt.plot(x1_values, y1_values,  marker = '.', color='b', label='originals Data Points', linewidth=1.0)
#     # plt.plot(x2_values, y2_values, linestyle='dotted', color='r', label='calculated Data Points', linewidth=1.0)
    
#     plt.title('Plot of states')
#     plt.xlabel('X Axis')
#     plt.ylabel('Y Axis')
#     plt.legend()

#     plt.show()
    

# if __name__ == '__main__':   
#     plot_from_files('states.txt', 'next_states.txt')


import matplotlib.pyplot as plt
import math

def car_bounding_box_vertices(width, length, angle):
    x1 = (length/2) * math.cos(angle) - (width/2) * math.sin(angle)
    y1 = (width/2) * math.cos(angle) + (length/2) * math.sin(angle)
    x2 = -(length/2) * math.cos(angle) - (width/2) * math.sin(angle)
    y2 = (width/2) * math.cos(angle) - (length/2) * math.sin(angle)
    x3 = (length/2) * math.cos(angle) + (width/2) * math.sin(angle)
    y3 = -(width/2) * math.cos(angle) + (length/2) * math.sin(angle)
    x4 = -(length/2) * math.cos(angle) + (width/2) * math.sin(angle)
    y4 = -(width/2) * math.cos(angle) - (length/2) * math.sin(angle)
    car_bounding_box_vertices = [(x1, y1), (x2, y2), (x3, y3), (x4, y4)]
    return car_bounding_box_vertices

# Example values
width = 4
length = 8
angle = math.radians(60)  # 30 degrees converted to radians

# Calculate bounding box vertices
vertices = car_bounding_box_vertices(width, length, angle)

# Plotting
car_x = [vertices[i][0] for i in range(4)] + [vertices[0][0]]
car_y = [vertices[i][1] for i in range(4)] + [vertices[0][1]]

plt.figure(figsize=(8, 8))
plt.plot(car_x, car_y, label='Car')
plt.scatter(*zip(*vertices), color='red', label='Bounding Box Vertices')

plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.title('Car Bounding Box with Rotated Vertices')
plt.legend()
plt.grid(True)
plt.show()



