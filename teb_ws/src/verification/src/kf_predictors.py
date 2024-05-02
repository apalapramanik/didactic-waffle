#!/usr/bin/env python

import cv2
import numpy as np
from filterpy.monte_carlo import systematic_resample
from filterpy.kalman import KalmanFilter
from filterpy.kalman import ExtendedKalmanFilter
from filterpy.kalman import UnscentedKalmanFilter
from filterpy.kalman import MerweScaledSigmaPoints
from filterpy.kalman import EnsembleKalmanFilter
import pandas as pd
# from marker_publisher import marker

class BaseFilter:
    def __init__(self):
        pass

    def predict_correct(self, x, y):
        pass
    
class KalmanFilterEstimator(BaseFilter):
    def __init__(self):
        # Initialize Kalman filter parameters
        self.kalman = cv2.KalmanFilter(4, 2)  # 4 states (x, y, vx, vy), 2 measurements (x, y)

        # Set the initial state covariance matrix
        # self.kalman.errorCovPost = np.eye(4, dtype=np.float32) * 1e-3

        # Measurement matrix: maps the predicted state to the measurements (x, y)
        self.kalman.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)

        # Transition matrix: maps the previous state to the predicted state (including velocities)
        self.kalman.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)

    def predict_correct(self, x, y):
        # Predict the next state
        predicted_state = self.kalman.predict()

        # Update the state based on new measurements
        measurement = np.array([[x], [y]], np.float32)
        self.kalman.correct(measurement)

        # Access the updated state
        updated_state = self.kalman.statePost
        predicted_x, predicted_y = updated_state[0, 0], updated_state[1, 0]

        return predicted_x, predicted_y
    

    

class ExtendedKalmanFilterEstimator(BaseFilter):
    def __init__(self):
        # Define the state transition function (linear motion model)
        self.dt = 0.25  # Time step
        self.ekf = ExtendedKalmanFilter(dim_x=4, dim_z=2)  # 4 states (x, y, vx, vy), 2 measurements (x, y)
        self.ekf.x = np.array([50, 50, 0, 0])  # Initial state [x, y, vx, vy]
        self.ekf.F = np.array([[1, 0, self.dt, 0],
                              [0, 1, 0, self.dt],
                              [0, 0, 1, 0],
                              [0, 0, 0, 1]])  # State transition matrix

    def state_transition_function(self, x):
        # State transition function for the Extended Kalman Filter
        # Assume constant velocity model with 4 states (x, y, vx, vy)
        F = np.array([[1, 0, self.dt, 0],
                      [0, 1, 0, self.dt],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])
        return np.dot(F, x)

    def measurement_function(self, x):
        # Measurement function for the Extended Kalman Filter (linear measurement model)
        return np.array([x[0], x[1]])  # Measurement is just X and Y

    def measurement_jacobian(self, x):
        # Jacobian of the measurement function for the Extended Kalman Filter
        return np.array([[1, 0, 0, 0],
                         [0, 1, 0, 0]])  # Measurement matrix for both X and Y

    def predict_correct(self, x, y):
        # Predict the next state
        self.ekf.predict()

        # Update with the new measurements (X and Y)
        z = np.array([x, y])
        HJacobian = self.measurement_jacobian(self.ekf.x)
        Hx = self.measurement_function(self.ekf.x)
        self.ekf.update(z, HJacobian=self.measurement_jacobian, Hx=self.measurement_function)

        # Access the updated state
        predicted_x, predicted_y = self.ekf.x[0], self.ekf.x[1]
        predicted_vx, predicted_vy = self.ekf.x[2], self.ekf.x[3]

        return predicted_x, predicted_y

class UnscentedKalmanFilterEstimator(BaseFilter):
    def __init__(self):
        # Define the state transition function (constant velocity model)
        self.dt = 0.25 # Time step
        self.ukf = None  # Initialize UKF instance

    def state_transition_function(self, x, dt):  # Add dt as an argument
        # State transition function for the Unscented Kalman Filter
        F = np.array([[1, 0, dt, 0],
                      [0, 1, 0, dt],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])
        return np.dot(F, x)

    def measurement_function(self, x):
        # Measurement function for the Unscented Kalman Filter (linear measurement model for both X and Y)
        H = np.array([[1, 0, 0, 0],
                      [0, 1, 0, 0]])  # Measurement matrix for both X and Y
        return np.dot(H, x)

    def predict_correct(self, x, y):
        # Predict the next state
        self.ukf.predict()

        # Update with the new measurements (X and Y)
        z = np.array([x, y])
        self.ukf.update(z)

        # Access the updated state
        predicted_x, predicted_y = self.ukf.x[0], self.ukf.x[1]
        predicted_vx, predicted_vy = self.ukf.x[2], self.ukf.x[3]

        return predicted_x, predicted_y
 
class EnsembleKalmanFilterEstimator(BaseFilter):
    def __init__(self):
        # Define the state transition function (constant velocity model)
        self.dt = 0.25  # Time step
        self.enkf = None  # Initialize EnKF instance

    def state_transition_function(self, x, dt):
        # State transition function for the Ensemble Kalman Filter
        F = np.array([[1, 0, dt, 0],
                      [0, 1, 0, dt],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])
        return np.dot(F, x)

    def measurement_function(self, x):
        # Measurement function for the Ensemble Kalman Filter (linear measurement model for both X and Y)
        H = np.array([[1, 0, 0, 0],
                      [0, 1, 0, 0]])  # Measurement matrix for both X and Y
        return np.dot(H, x)

    def predict_correct(self, x, y):
        # Predict the next state
        self.enkf.predict()

        # Update with the new measurements (X and Y)
        z = np.array([x, y])
        self.enkf.update(z)

        # Access the updated state
        predicted_x, predicted_y = self.enkf.x[0], self.enkf.x[1]
        predicted_vx, predicted_vy = self.enkf.x[2], self.enkf.x[3]

        return predicted_x, predicted_y


class FilterEstimator:
    def __init__(self, prediction_points, steps):
        self.prediction_points = prediction_points
        self.steps = steps
        

    
    
    def kf_caller1(self):
        kf_estimator = KalmanFilterEstimator()
        self.predictions_array = []
        self.error = 0.0
        self.pred_array1 = []
        self.points_array = []
        self.error_array = []
        
        for x, y,z in self.prediction_points:
            # kf_estimator = KalmanFilterEstimator()
            predicted_x, predicted_y = kf_estimator.predict_correct(x, y)
            print(f"Initial Point (x, y) = ({x}, {y})")
            print(f"KF Predicted Point:", predicted_x, predicted_y) 
            self.points_array.append([x,y])           
            self.pred_array1.append([predicted_x, predicted_y])            
            self.error = self.euclidean_distance(x, y, predicted_x, predicted_y)
            self.error_array.append(self.error)
            
        
            
        # np.savetxt("pred_kf2.txt", self.pred_array1, delimiter=",")
        # np.savetxt("org2.txt", self.points_array, delimiter=",")
        # np.savetxt("error_kf.txt", self.error_array)
        # print("error:", self.error)
        
         # Save data to the files in append mode
        with open("pred_kf_h1.txt", "a") as f:
            np.savetxt(f, self.pred_array1, delimiter=",")

        with open("org_h1.txt", "a") as f:
            np.savetxt(f, self.points_array, delimiter=",")

        with open("error_kf_h1.txt", "a") as f:
            np.savetxt(f, [self.error_array], delimiter=",")

        print("error:", self.error)
        
        for i in range(self.steps):
            predicted_x, predicted_y = kf_estimator.predict_correct(predicted_x, predicted_y)
            self.predictions_array.append([predicted_x, predicted_y])
            print(f"Point {i + 1}: (x, y) = ({predicted_x}, {predicted_y})")
  
        return self.predictions_array, self.error
    
    def kf_caller2(self):
        kf_estimator2 = KalmanFilterEstimator()
        self.predictions_array = []
        self.error = 0.0
        self.pred_array1 = []
        self.points_array = []
        self.error_array = []
        
        for x, y,z in self.prediction_points:
            # kf_estimator = KalmanFilterEstimator()
            predicted_x, predicted_y = kf_estimator2.predict_correct(x, y)
            print(f"Initial Point (x, y) = ({x}, {y})")
            print(f"KF Predicted Point:", predicted_x, predicted_y) 
            self.points_array.append([x,y])           
            self.pred_array1.append([predicted_x, predicted_y])            
            self.error = self.euclidean_distance(x, y, predicted_x, predicted_y)
            self.error_array.append(self.error)
            
        
            
        # np.savetxt("pred_kf2.txt", self.pred_array1, delimiter=",")
        # np.savetxt("org2.txt", self.points_array, delimiter=",")
        # np.savetxt("error_kf.txt", self.error_array)
        # print("error:", self.error)
        
         # Save data to the files in append mode
        with open("pred_kf_h2.txt", "a") as f:
            np.savetxt(f, self.pred_array1, delimiter=",")

        with open("org_h2.txt", "a") as f:
            np.savetxt(f, self.points_array, delimiter=",")

        with open("error_kf_h2.txt", "a") as f:
            np.savetxt(f, [self.error_array], delimiter=",")

        print("error:", self.error)
        
        for i in range(self.steps):
            predicted_x, predicted_y = kf_estimator2.predict_correct(predicted_x, predicted_y)
            self.predictions_array.append([predicted_x, predicted_y])
            print(f"Point {i + 1}: (x, y) = ({predicted_x}, {predicted_y})")
  
        return self.predictions_array, self.error
    
   
    
    def ekf_caller(self):  
    
        # Create ExtendedKalmanFilterEstimator instance
        ekf_estimator = ExtendedKalmanFilterEstimator()
        self.predictions_array = []
        self.pred_array1 = []
        self.points_array = []
        self.error_array = []
        
        self.error = 0.0

        # Loop for each point in the input array
        for x, y in self.prediction_points:
            # print(f"Point: ({x}, {y})")
            self.points_array.append([x,y])
            # Extended Kalman Filter
            ekf_predicted_x, ekf_predicted_y = ekf_estimator.predict_correct(x, y)
            # print(f"EKF Predicted state after correction: (x, y) = ({ekf_predicted_x}, {ekf_predicted_y})")       
            self.pred_array1.append([ekf_predicted_x, ekf_predicted_y])            
            self.error = self.euclidean_distance(x, y, ekf_predicted_x, ekf_predicted_y)
            self.error_array.append(self.error)
            
        # np.savetxt("error_ekf.txt", self.error_array)   
        # np.savetxt("pred_ekf2.txt", self.pred_array1, delimiter=",")
        # np.savetxt("org_ekf2.txt", self.points_array, delimiter=",")
        
         # Save data to the files in append mode
        with open("pred_ekf2.txt", "a") as f:
            np.savetxt(f, self.pred_array1, delimiter=",")

        # with open("org2.txt", "a") as f:
        #     np.savetxt(f, self.points_array, delimiter=",")

        with open("error_ekf.txt", "a") as f:
            np.savetxt(f, [self.error_array], delimiter=",")

        print("error:", self.error)
        
            # Generate 5 more predictions using the Extended Kalman Filter
        for _ in range(self.steps):
            ekf_predicted_x, ekf_predicted_y = ekf_estimator.predict_correct(ekf_predicted_x, ekf_predicted_y)
            self.predictions_array.append([ekf_predicted_x, ekf_predicted_y])
            # print(f"Additional EKF Prediction: (x, y) = ({ekf_predicted_x}, {ekf_predicted_y})")
        return self.predictions_array, self.error
        
   
   
   
   
    def ukf_caller(self):
    
        
        # Create UnscentedKalmanFilterEstimator instance
        ukf_estimator = UnscentedKalmanFilterEstimator()
        self.predictions_array = []
        self.points_array = []
        self.pred_array1 = []
        self.error = 0.0
        self.error_array = []

        # Initialize UKF with sigma points
        sigma = 0.1
        points = MerweScaledSigmaPoints(n=4, alpha=sigma, beta=2, kappa=0)

        # Define process noise covariance (Q) and measurement noise covariance (R)
        Q = np.diag([0.01, 0.01, 0.01, 0.01])  # Process noise covariance
        R = np.diag([0.1, 0.1])  # Measurement noise covariance

        # Initialize UKF
        ukf_estimator.ukf = UnscentedKalmanFilter(dim_x=4, dim_z=2, dt=0.25,
                                                hx=ukf_estimator.measurement_function,
                                                fx=ukf_estimator.state_transition_function,
                                                points=points)
        ukf_estimator.ukf.x = np.array([self.prediction_points[0][0], self.prediction_points[0][1], 0, 0])  # Initial state [x, y, vx, vy]
        ukf_estimator.ukf.Q = Q
        ukf_estimator.ukf.R = R

        # Loop for each point in the input array
        for x, y in self.prediction_points:
            print(f"Point: ({x}, {y})")
            self.points_array.append([x,y])
            #Unscented Kalman Filter
            ukf_predicted_x, ukf_predicted_y = ukf_estimator.predict_correct(x, y)
            print(f"UKF Predicted state after correction: (x, y) = ({ukf_predicted_x}, {ukf_predicted_y})")            
            self.pred_array1.append([ukf_predicted_x, ukf_predicted_y])            
            self.error = self.euclidean_distance(x, y, ukf_predicted_x, ukf_predicted_y)
            self.error_array.append(self.error)
        
        # np.savetxt("error_ukf.txt", self.error_array)    
        # np.savetxt("pred_ukf2.txt", self.pred_array1, delimiter=",")
        # np.savetxt("org_ukf2.txt", self.points_array, delimiter=",")
        # print("error:", self.error)
        
         # Save data to the files in append mode
        # with open("pred_ukf2.txt", "a") as f:
        #     np.savetxt(f, self.pred_array1, delimiter=",")
        # Save data to the files in append mode
        with open( f"{base_folder}/pred_ukf.txt", "a") as f:
            np.savetxt(f, self.pred_array1, delimiter=",")
            
            # f"{base_folder}/always_human1.txt"

        # with open("org2.txt", "a") as f:
        #     np.savetxt(f, self.points_array, delimiter=",")

        with open("error_ukf.txt", "a") as f:
            np.savetxt(f, [self.error_array], delimiter=",")

        print("error:", self.error)
        
    
        # Generate 5 more predictions using the Unscented Kalman Filter
        for _ in range(self.steps):
         
            ukf_predicted_x, ukf_predicted_y = ukf_estimator.predict_correct(ukf_predicted_x, ukf_predicted_y)
            self.predictions_array.append( [ukf_predicted_x, ukf_predicted_y])
            print(f"Additional UKF Prediction: (x, y) = ({ukf_predicted_x}, {ukf_predicted_y})")
        return self.predictions_array, self.error
    
    
    
    
    
    
    def enkf_caller(self):
        # Create EnsembleKalmanFilterEstimator instance
        enkf_estimator = EnsembleKalmanFilterEstimator()
        self.predictions_array = []
        self.pred_array1 = []
        self.points_array = []
        self.error = 0.0
        self.error_array = []

        # Initialize EnKF
        state_size = 4  # State size [x, y, vx, vy]
        measurement_size = 2  # Measurement size (X and Y coordinates)
        ensemble_size = 200  # Number of ensemble members

        # Set the initial state and covariance
        initial_state = np.array([self.prediction_points[0][0], self.prediction_points[0][1], 0, 0])
        initial_covariance = np.eye(state_size) * 0.1

        enkf_estimator.enkf = EnsembleKalmanFilter(x=initial_state, P=initial_covariance, dim_z=measurement_size,
                                                dt=0.25, N=ensemble_size, hx=enkf_estimator.measurement_function,
                                                fx=enkf_estimator.state_transition_function)

        # Define process noise covariance (Q) and measurement noise covariance (R)
        enkf_estimator.enkf.Q = np.diag([0.01, 0.01, 0.01, 0.01])  # Process noise covariance
        enkf_estimator.enkf.R = np.diag([0.1, 0.1])  # Measurement noise covariance

        # Loop for each point in the input array
        for x, y in self.prediction_points:
            # print(f"Point: ({x}, {y})")
            self.points_array.append([x,y])
            # Ensemble Kalman Filter
            enkf_predicted_x, enkf_predicted_y = enkf_estimator.predict_correct(x, y)
            # print(f"EnKF Predicted state after correction: (x, y) = ({enkf_predicted_x}, {enkf_predicted_y})")
            self.pred_array1.append([ enkf_predicted_x, enkf_predicted_y])           
            self.error = self.euclidean_distance(x, y, enkf_predicted_x, enkf_predicted_y)
            self.error_array.append(self.error)
            
        # np.savetxt("error_enkf.txt", self.error_array)    
        # np.savetxt("pred_enkf2.txt", self.pred_array1, delimiter=",")
        # np.savetxt("org_enkf2.txt", self.points_array, delimiter=",")        
        # print("error:", error)
        
        
         # Save data to the files in append mode
        with open("pred_enkf2.txt", "a") as f:
            np.savetxt(f, self.pred_array1, delimiter=",")

        # with open("org2.txt", "a") as f:
        #     np.savetxt(f, self.points_array, delimiter=",")

        with open("error_enkf.txt", "a") as f:
            np.savetxt(f, [self.error_array], delimiter=",")

        print("error:", self.error)
        
        
        # Generate 5 more predictions using the Ensemble Kalman Filter
        for _ in range(self.steps):
            enkf_predicted_x, enkf_predicted_y = enkf_estimator.predict_correct(enkf_predicted_x, enkf_predicted_y)
            self.predictions_array.append([enkf_predicted_x, enkf_predicted_y])
            # print(f"Additional EnKF Prediction: (x, y) = ({enkf_predicted_x}, {enkf_predicted_y})")
        return self.predictions_array, self.error
        
    def euclidean_distance(self, x1, y1, x2, y2):
        return np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
       
    def pred(self, filter_type):
        if filter_type == "kf":
            print("Kalman Filter:")
            self.kf_caller()

        elif filter_type == "ekf":
            print("\nExtended Kalman Filter:")
            self.ekf_caller()

        elif filter_type == "ukf":
            print("\nUnscented Kalman Filter:")
            self.ukf_caller()

        elif filter_type == "enkf":
            print("\nEnsemble Kalman Filter:")
            self.enkf_caller()

        else:
            print("Invalid filter type!")



    
if __name__ == "__main__":
    
    base_folder = 'evaluation3/location5/h3'
    prediction_points = []
    with open( f"{base_folder}/org_h1.txt", 'r') as file:
        for line in file:
            x, y = map(float, line.strip().split(','))  # Split by comma and convert values to float
            prediction_points.append((x, y))

    # print(prediction_points)  # Optional: Print to verify the data      
    steps = 5
    # prediction_points = [(50, 50), (100, 100), (150, 100), (200, 100)]
    # Create an instance of FilterEstimator
    filter_estimator = FilterEstimator(prediction_points, steps)

    # Call the main function with the desired filter type
    filter_type = "ukf"  # Change this to the desired filter type
    filter_estimator.pred(filter_type)








