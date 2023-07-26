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

# Calculate RMSE for each method
rmse_kf = np.sqrt(np.mean(error_kf**2))

rmse_ekf = np.sqrt(np.mean(error_ekf**2))
rmse_ukf = np.sqrt(np.mean(error_ukf**2))
print(rmse_kf)
rmse_enkf = np.sqrt(np.mean(error_enkf**2))

# Create a single set of axes for plotting RMSE
fig, ax = plt.subplots(figsize=(8, 6))

# Plot the RMSE values for each method using a bar plot
methods = ['KF', 'EKF', 'UKF', 'EnKF']
rmse_values = [rmse_kf, rmse_ekf, rmse_ukf, rmse_enkf]
bars = ax.bar(methods, rmse_values, color=['tab:blue', 'tab:green', 'tab:red', 'tab:purple'])

# Set title, labels, and grid
ax.set_title('Root Mean Squared Error (RMSE) for Each Method', fontsize=18, fontweight='bold')
ax.set_xlabel('Method', fontsize=20, fontweight='bold')
ax.set_ylabel('RMSE (meters)', fontsize=20, fontweight='bold')
# Increase the font size for tick labels on both x and y axes
ax.tick_params(axis='both', which='major', labelsize=20)
ax.grid(axis='y', linestyle='--', alpha=0.7)

# # Add text annotations on top of each bar
# for bar in bars:
#     height = bar.get_height()
#     ax.annotate(f'{height:.2f}', xy=(bar.get_x() + bar.get_width() / 2, height),
#                 xytext=(0, 3),  # 3 points vertical offset
#                 textcoords="offset points",
#                 ha='center', va='bottom', fontsize=12, fontweight='bold')
    
# Add text annotations on top of each bar
for bar in bars:
    height = bar.get_height()
    ax.annotate(f'{height:.4f}m', xy=(bar.get_x() + bar.get_width() / 2, height),
                xytext=(0, 3),  # 3 points vertical offset
                textcoords="offset points",
                ha='center', va='bottom', fontsize=18, fontweight='bold')



# Show the plot
plt.show()