import numpy as np
import math

class KalmanFilter:
    def __init__(self):
        # Initialize state variables
        self.mu = np.zeros(3)  # State mean: [x, y, heading_angle]
        self.sigma = np.eye(3)  # State covariance

        # Process noise covariance matrix
        self.Q = np.diag([0.01, 0.01, 0.01])  # Tune according to process noise

        # Observation noise covariance matrix
        self.R = np.diag([0.01, 0.01])  # Tune according to observation noise

        # State transition matrix
        self.F = np.eye(3)  # Identity matrix for this example

        # Observation matrix (extracts x, y from state)
        self.H = np.array([[1, 0, 0],
                           [0, 1, 0]])

    def predict(self, dt):
        # Update state transition matrix based on time step
        self.F[0, 2] = dt * math.cos(self.mu[2])  # Update x
        self.F[1, 2] = dt * math.sin(self.mu[2])  # Update y

        # Predict next state using motion model
        self.mu = np.dot(self.F, self.mu)
        self.sigma = np.dot(np.dot(self.F, self.sigma), self.F.T) + self.Q

    def update(self, measurement):
        # Calculate Kalman gain
        K = np.dot(np.dot(self.sigma, self.H.T), np.linalg.inv(np.dot(np.dot(self.H, self.sigma), self.H.T) + self.R))

        # Update state estimate
        innovation = measurement - np.dot(self.H, self.mu)
        self.mu = self.mu + np.dot(K, innovation)

        # Update covariance matrix
        self.sigma = np.dot((np.eye(3) - np.dot(K, self.H)), self.sigma)

    def process_measurement(self, measurement, dt):
        # Perform prediction step
        self.predict(dt)

        # Perform update step
        self.update(measurement)

# Example usage:
if __name__ == "__main__":
    kf = KalmanFilter()

    # Measurement (x, y)
    measurement = np.array([self.x_mean, self.y_mean])

    # Time step
    dt = 0.13  # Assuming time step is constant, as in the provided code

    # Process measurement
    kf.process_measurement(measurement, dt)

    # Get estimated state
    estimated_state = kf.mu
    print("Estimated state:", estimated_state)
