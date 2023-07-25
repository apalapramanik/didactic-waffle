import numpy as np
import matplotlib.pyplot as plt
from sklearn.metrics import mean_squared_error

original_points = np.loadtxt("org_ukf2.txt", delimiter=",")
predicted_points = np.loadtxt("pred_ukf2.txt", delimiter=",")



x = [p[0] for p in original_points[5:]]
y = [p[1] for p in original_points[5:]]

w = [m[0] for m in predicted_points[5:]]
z = [m[1] for m in predicted_points[5:]]

# w = [m[0] for m in predicted_points[1:]]
# z = [m[1] for m in predicted_points[1:]]

org_cols = original_points[5:,:2]
pred_cols = predicted_points[5:,:2]


# Calculate the RMSE
rmse = np.sqrt(mean_squared_error(org_cols, pred_cols))
# print(rmse)

# Add and subtract the error from the original values to find the upper and lower bounds
upper_bound = org_cols + rmse
lower_bound = org_cols - rmse

# Plot the original points, predicted points, lower bound, and upper bound
plt.plot(x, y, 'r-', label='Original Trajectory', linewidth=5)
# plt.plot(w, z, 'g-', label='Predicted Points', linewidth=2)
plt.plot(upper_bound[:, 0], upper_bound[:, 1], linestyle = '--',color = 'tab:blue', label='Upper Bound', linewidth=5)
plt.plot(lower_bound[:, 0], lower_bound[:, 1],  linestyle = '--',color = 'tab:green', label='Lower Bound', linewidth=5)
# plt.fill_between(org_cols[:,0], upper_bound[:,1], lower_bound[:,1], color='tab:gray', alpha=0.4)
plt.xlabel('X-coordinate', fontsize=20,fontweight='bold')
plt.ylabel('Z-coordinate', fontsize=20,fontweight='bold')
plt.legend(loc='upper left',fontsize=20, frameon=False)
plt.gca().tick_params(axis='both', which='major', labelsize=30)
plt.title('Unscented Kalman Filter Root Mean Square Error Plot', fontsize=20,fontweight='bold')
plt.show()