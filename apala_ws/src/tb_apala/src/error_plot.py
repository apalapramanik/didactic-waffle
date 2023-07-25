import numpy as np
import matplotlib.pyplot as plt

# Function to read error data from a file
def read_error_data(file_path):
    return np.genfromtxt(file_path)

# Load error data from files
error_kf = read_error_data("error_kf.txt")
error_ekf = read_error_data("error_ekf.txt")
error_ukf = read_error_data("error_ukf.txt")
error_enkf = read_error_data("error_enkf.txt")

# Create four subplots
fig, axes = plt.subplots(2, 2, figsize=(12, 10))
fig.suptitle('Error Plots', fontsize=20)

# Plot the data in each subplot with different colors and linewidths
axes[0, 0].plot(error_kf, label='KF', color='tab:blue', linewidth=5)
axes[0, 0].set_title('Error in KF', fontsize=16, fontweight='bold')
axes[0, 0].set_xlabel('Time Step', fontsize=12, fontweight='bold')
axes[0, 0].set_ylabel('Error', fontsize=12, fontweight='bold')
axes[0, 0].legend(fontsize=12)

axes[0, 1].plot(error_ekf, label='EKF', color='tab:green', linewidth=5)
axes[0, 1].set_title('Error in EKF', fontsize=16, fontweight='bold')
axes[0, 1].set_xlabel('Time Step', fontsize=12, fontweight='bold')
axes[0, 1].set_ylabel('Error', fontsize=12, fontweight='bold')
axes[0, 1].legend(fontsize=12)

axes[1, 0].plot(error_ukf, label='UKF', color='tab:red', linewidth=5)
axes[1, 0].set_title('Error in UKF', fontsize=16, fontweight='bold')
axes[1, 0].set_xlabel('Time Step', fontsize=12, fontweight='bold')
axes[1, 0].set_ylabel('Error', fontsize=12, fontweight='bold')
axes[1, 0].legend(fontsize=12)

axes[1, 1].plot(error_enkf, label='EnKF', color='tab:purple', linewidth=5)
axes[1, 1].set_title('Error in EnKF', fontsize=16, fontweight='bold')
axes[1, 1].set_xlabel('Time Step', fontsize=12, fontweight='bold')
axes[1, 1].set_ylabel('Error', fontsize=12, fontweight='bold')
axes[1, 1].legend(fontsize=12)

# Adjust layout and add space between subplots
plt.tight_layout(rect=[0, 0.03, 1, 0.95])

# Show the plots
plt.show()
