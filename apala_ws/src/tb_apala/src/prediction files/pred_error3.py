

import numpy as np
import matplotlib.pyplot as plt

# Function to read error data from a file
def read_error_data(file_path):
    return np.genfromtxt(file_path)

# Load error data from files
error_kf = read_error_data("error_kf.txt")[5:]  # Exclude the first four time steps
error_ekf = read_error_data("error_ekf.txt")[5:]  # Exclude the first four time steps
error_ukf = read_error_data("error_ukf.txt")[5:]  # Exclude the first four time steps
error_enkf = read_error_data("error_enkf.txt")[5:]  # Exclude the first four time steps

# Create a single set of axes
fig, ax = plt.subplots(figsize=(12, 10))
# fig.suptitle('Error Plots', fontsize=20)

# Plot the data in the same graph with different colors and linewidths
ax.plot(error_kf, label='KF', color='tab:blue', linewidth=5)
# ax.plot(error_ekf, label='EKF', color='tab:green', linewidth=5)
ax.plot(error_ukf, label='UKF', color='tab:red', linewidth=5)
ax.plot(error_enkf, label='EnKF', color='tab:purple', linewidth=5)

# Set title, labels, and legend
ax.set_title('Error Plots', fontsize=18, fontweight='bold')
ax.set_xlabel('Time Step', fontsize=18, fontweight='bold')
ax.set_ylabel('Error', fontsize=18, fontweight='bold')
# Increase the font size for tick labels on both x and y axes
ax.tick_params(axis='both', which='major', labelsize=18)
ax.legend(fontsize=20)



# Show the plot
plt.show()