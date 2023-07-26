import numpy as np
import matplotlib.pyplot as plt
from sklearn.metrics import mean_squared_error

original_points = np.loadtxt("org2.txt", delimiter=",")
predicted_points_kf = np.loadtxt("pred_kf2.txt", delimiter=",")
predicted_points_ekf = np.loadtxt("pred_ekf2.txt", delimiter=",")
predicted_points_ukf = np.loadtxt("pred_ukf2.txt", delimiter=",")
predicted_points_enkf = np.loadtxt("pred_enkf2.txt", delimiter=",")





x = [p[0] for p in original_points[5:]]
y = [p[1] for p in original_points[5:]]

w_kf = [m[0] for m in predicted_points_kf[5:]]
z_kf = [m[1] for m in predicted_points_kf[5:]]

w_ekf = [m[0] for m in predicted_points_ekf[5:]]
z_ekf = [m[1] for m in predicted_points_ekf[5:]]

w_ukf = [m[0] for m in predicted_points_ukf[5:]]
z_ukf = [m[1] for m in predicted_points_ukf[5:]]

w_enkf = [m[0] for m in predicted_points_enkf[5:]]
z_enkf = [m[1] for m in predicted_points_enkf[5:]]

# w = [m[0] for m in predicted_points[1:]]
# z = [m[1] for m in predicted_points[1:]]

org_cols = original_points[5:,:2]
pred_cols_kf = predicted_points_kf[5:,:2]
pred_cols_ekf = predicted_points_ekf[5:,:2]
pred_cols_ukf = predicted_points_ukf[5:,:2]
pred_cols_enkf = predicted_points_enkf[5:,:2]


# Calculate the RMSE
rmse_kf = np.sqrt(mean_squared_error(org_cols, pred_cols_kf))
rmse_ekf = np.sqrt(mean_squared_error(org_cols, pred_cols_ekf))
rmse_ukf = np.sqrt(mean_squared_error(org_cols, pred_cols_ukf))
rmse_enkf = np.sqrt(mean_squared_error(org_cols, pred_cols_enkf))

# print(rmse)

# Add and subtract the error from the original values to find the upper and lower bounds
upper_bound_kf = org_cols + rmse_kf
lower_bound_kf = org_cols - rmse_kf

upper_bound_ekf = org_cols + rmse_ekf
lower_bound_ekf = org_cols - rmse_ekf

upper_bound_ukf = org_cols + rmse_ukf
lower_bound_ukf = org_cols - rmse_ukf

upper_bound_enkf = org_cols + rmse_enkf
lower_bound_enkf = org_cols - rmse_enkf

# Plot the original points, predicted points, lower bound, and upper bound
plt.plot(x, y, 'r-', label='Original Trajectory', linewidth=5)
# plt.plot(w, z, 'g-', label='Predicted Points', linewidth=2)


plt.plot(upper_bound_kf[:, 0], upper_bound_kf[:, 1], linestyle = '--',color = 'tab:blue', label='Upper Bound KF', linewidth=5)
plt.plot(lower_bound_kf[:, 0], lower_bound_kf[:, 1],  linestyle = '--',color = 'tab:blue', label='Lower Bound KF', linewidth=5)

# plt.plot(upper_bound_ekf[:, 0], upper_bound_ekf[:, 1], linestyle = '--',color = 'tab:green', label='Upper Bound EKF', linewidth=5)
# plt.plot(lower_bound_ekf[:, 0], lower_bound_ekf[:, 1],  linestyle = '--',color = 'tab:green', label='Lower Bound EKF', linewidth=5)

plt.plot(upper_bound_ukf[:, 0], upper_bound_ukf[:, 1], linestyle = '--',color = 'tab:purple', label='Upper Bound UKF', linewidth=5)
plt.plot(lower_bound_ukf[:, 0], lower_bound_ukf[:, 1],  linestyle = '--',color = 'tab:purple', label='Lower Bound UKF', linewidth=5)

plt.plot(upper_bound_enkf[:, 0], upper_bound_enkf[:, 1], linestyle = '--',color = 'tab:olive', label='Upper Bound ENKF', linewidth=5)
plt.plot(lower_bound_enkf[:, 0], lower_bound_enkf[:, 1],  linestyle = '--',color = 'tab:olive', label='Lower Bound ENKF', linewidth=5)




# plt.fill_between(org_cols[:,0], upper_bound[:,1], lower_bound[:,1], color='tab:gray', alpha=0.4)
plt.xlabel('X-coordinate', fontsize=20,fontweight='bold')
plt.ylabel('Z-coordinate', fontsize=20,fontweight='bold')
plt.legend(loc='upper left',fontsize=20, frameon=False)
plt.gca().tick_params(axis='both', which='major', labelsize=30)
plt.title(' Root Mean Square Error Plot', fontsize=20,fontweight='bold')
plt.show()