import cv2
import numpy as np
from filterpy.monte_carlo import systematic_resample
from filterpy.kalman import ExtendedKalmanFilter
from filterpy.kalman import UnscentedKalmanFilter
from filterpy.kalman import MerweScaledSigmaPoints
from filterpy.kalman import EnsembleKalmanFilter

#*************** KALMAN FILTER *******************************************************************************************

# Initialize Kalman filter parameters
kalman = cv2.KalmanFilter(4, 2)  # 4 states (x, y, vx, vy), 2 measurements (x, y)
kalman.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
kalman.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)

# Real-time prediction
prediction_points = [(50, 50), (100, 100), (150, 100), (200, 100)]

for point in prediction_points:
    new_x, new_y = point

    # Predict the next state
    predicted_state = kalman.predict()

    # Update the state based on new measurements
    measurement = np.array([[new_x], [new_y]], np.float32)
    kalman.correct(measurement)

    # Access the updated state
    updated_state = kalman.statePost
    predicted_x, predicted_y = updated_state[0, 0], updated_state[1, 0]
    
    # print(f"Predicted (x, y, vx, vy) for ({new_x}, {new_y}): ({predicted_x}, {predicted_y}, {predicted_vx}, {predicted_vy})")

    
print(f"KF Predicted (x, y) for ({new_x}, {new_y}): ({predicted_x}, {predicted_y})")


    
#************************* EXTENDED KALMAN FILTER **********************************************************************
    


# Define the state transition function (linear motion model)
def state_transition_function(x, dt):
    # Assume constant velocity model with 4 states (x, y, vx, vy)
    F = np.array([[1, 0, dt, 0],
                  [0, 1, 0, dt],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])
    return np.dot(F, x)

# Define the measurement function (linear measurement model)
def measurement_function(x):
    return np.array([x[0], x[1]])  # Measurement is just X and Y

# Define the Jacobian of the measurement function (HJacobian)
def measurement_jacobian(x):
    H = np.array([[1, 0, 0, 0],
                  [0, 1, 0, 0]])  # Measurement matrix for both X and Y
    return H

# Initialize EKF
ekf = ExtendedKalmanFilter(dim_x=4, dim_z=2)  # 4 states (x, y, vx, vy), 2 measurements (x, y)
ekf.x = np.array([50, 50, 0, 0])  # Initial state [x, y, vx, vy]
ekf.F = np.array([[1, 0, 1, 0],
                  [0, 1, 0, 1],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])  # State transition matrix
ekf.H = measurement_jacobian

# Real-time prediction
prediction_points = [(50, 50), (100, 100), (150, 100), (200, 100)]
dt = 1.0  # Time step

for point in prediction_points:
    new_x, new_y = point

    # Predict the next state
    ekf.predict(dt)

    # Update with the new measurements (X and Y)
    z = np.array([new_x, new_y])
    ekf.update(z, HJacobian=measurement_jacobian, Hx=measurement_function)

    # Access the updated state
    predicted_x, predicted_y = ekf.x[0], ekf.x[1]
    predicted_vx, predicted_vy = ekf.x[2], ekf.x[3]

    # print(f"Predicted (x, y, vx, vy) for ({new_x}, {new_y}): ({predicted_x}, {predicted_y}, {predicted_vx}, {predicted_vy})")
    
print(f"EKF Predicted (x, y) for ({new_x}, {new_y}): ({predicted_x}, {predicted_y} )")

    
#*************************************UNSCENTED KLAMAN FILTER **********************************************************
    
# Define the state transition function (constant velocity model)
def state_transition_function(x, dt):
    F = np.array([[1, 0, dt, 0],
                  [0, 1, 0, dt],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])
    return np.dot(F, x)

# Define the measurement function (linear measurement model for both X and Y)
def measurement_function(x):
    H = np.array([[1, 0, 0, 0],
                  [0, 1, 0, 0]])  # Measurement matrix for both X and Y
    return np.dot(H, x)

# Initialize UKF with sigma points
sigma = 0.1
points = MerweScaledSigmaPoints(n=4, alpha=sigma, beta=2, kappa=0)

# Define process noise covariance (Q) and measurement noise covariance (R)
Q = np.diag([0.01, 0.01, 0.01, 0.01])  # Process noise covariance
R = np.diag([0.1, 0.1])  # Measurement noise covariance

# Initialize UKF
ukf = UnscentedKalmanFilter(dim_x=4, dim_z=2, dt=1.0, hx=measurement_function, fx=state_transition_function, points=points)
ukf.x = np.array([50, 50, 0, 0])  # Initial state [x, y, vx, vy]
ukf.Q = Q
ukf.R = R

# Real-time prediction
prediction_points = [(50, 50), (100, 100), (150, 100), (200, 100)]

for point in prediction_points:
    new_x, new_y = point

    # Predict the next state
    ukf.predict()

    # Update with the new measurements (X and Y)
    z = np.array([new_x, new_y])
    ukf.update(z)

    # Access the updated state
    predicted_x, predicted_y = ukf.x[0], ukf.x[1]
    predicted_vx, predicted_vy = ukf.x[2], ukf.x[3]

    # print(f"Predicted (x, y, vx, vy) for ({new_x}, {new_y}): ({predicted_x}, {predicted_y}, {predicted_vx}, {predicted_vy})")
    
print(f"UKF Predicted (x, y) for ({new_x}, {new_y}): ({predicted_x}, {predicted_y} )")

    
    
#************************************************* ENSEMBLE KALMAN FILTER **********************************************

# Define the state transition function (constant velocity model)
def state_transition_function(x, dt):
    F = np.array([[1, 0, dt, 0],
                  [0, 1, 0, dt],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])
    return np.dot(F, x)

# Define the measurement function (linear measurement model for both X and Y)
def measurement_function(x):
    H = np.array([[1, 0, 0, 0],
                  [0, 1, 0, 0]])  # Measurement matrix for both X and Y
    return np.dot(H, x)

# Initialize EnKF
state_size = 4  # State size [x, y, vx, vy]
measurement_size = 2  # Measurement size (X and Y coordinates)
ensemble_size = 100  # Number of ensemble members

# Set the initial state and covariance
initial_state = np.array([50, 50, 0, 0])
initial_covariance = np.eye(state_size) * 0.1

ekf = EnsembleKalmanFilter(x=initial_state, P=initial_covariance, dim_z=measurement_size,
                           dt=1.0, N=ensemble_size, hx=measurement_function, fx=state_transition_function)

# Define process noise covariance (Q) and measurement noise covariance (R)
ekf.Q = np.diag([0.01, 0.01, 0.01, 0.01])  # Process noise covariance
ekf.R = np.diag([0.1, 0.1])  # Measurement noise covariance

# Real-time prediction
prediction_points = [(50, 50), (100, 100), (150, 100), (200, 100)]
dt = 1.0  # Time step

for point in prediction_points:
    new_x, new_y = point

    # Predict the next state
    ekf.predict()

    # Update with the new measurements (X and Y)
    z = np.array([new_x, new_y])
    ekf.update(z)

    # Access the updated state
    predicted_x, predicted_y = ekf.x[0], ekf.x[1]
    predicted_vx, predicted_vy = ekf.x[2], ekf.x[3]

    # print(f"Predicted (x, y, vx, vy) for ({new_x}, {new_y}): ({predicted_x}, {predicted_y}, {predicted_vx}, {predicted_vy})")
    
print(f"EnKF Predicted (x, y) for ({new_x}, {new_y}): ({predicted_x}, {predicted_y} )")
    
