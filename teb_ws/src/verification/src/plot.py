import re
import matplotlib.pyplot as plt
import numpy as np

def plot_from_files(filename1, filename2):
    # Load data from file
    with open(filename1, 'r') as file:
        data1 = file.readlines()
        
    with open(filename2, 'r') as file:
        data2 = file.readlines()
    

    # Extract x and y values from each array
    x1_values = []
    y1_values = []
    
    x2_values = []
    y2_values = []
    
    

    for line1 in data1:
        match1 = re.findall(r"[-+]?\d*\.\d+", line1)
        if len(match1) >= 2:
            x1_values.append(float(match1[0]))
            y1_values.append(float(match1[1]))
            
    for line2 in data2:
        match2 = re.findall(r"[-+]?\d*\.\d+", line2)
        if len(match2) >= 2:
            x2_values.append(float(match2[0]))
            y2_values.append(float(match2[1]))

    # Plotting
    plt.figure(figsize=(10, 6))
    plt.plot(x1_values, y1_values, marker='o', linestyle='-', color='b', label='original Data Points', linewidth=0.25)
    plt.plot(x2_values, y2_values, marker='o', linestyle='-', color='r', label='calculated Data Points', linewidth=0.25)
    
    plt.title('Plot of states')
    plt.xlabel('X Axis')
    plt.ylabel('Y Axis')
    plt.legend()

    plt.show()
    

if __name__ == '__main__':   
    plot_from_files('states.txt', 'next_states.txt')


