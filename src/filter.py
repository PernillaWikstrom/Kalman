import numpy as np


class Covariance:
    def __init__(self, std: float):
        self.std = std
        self.variance = std**2.0


class LinearKalmanFilter:
    def __init__(self, dt: float):
        self.dt = dt
        self.nStates = 6
        self.nMeasurements = 2

    def initialize(
        self,
        initial_state_vector: np.array,
        std_measurements: float,
        std_process: float,
    ):

        # State transition matrix
        self.F = self.state_transition_matrix()
        # States
        self.state_vector = self.F @ initial_state_vector
        # State disturbance
        self.w = np.zeros_like(self.state_vector)

        # Process noise Covariance matrix
        self.Q = self.process_noise_matrix(std_process)
        # State Covariance matrix (error of estimate)
        self.P = self.initial_covariance_matrix()

        # Measurement Covariance matrix (error of measurement)
        cov = Covariance(std_measurements)
        self.R = np.eye(self.nMeasurements) * cov.variance

        # Observation matrix:
        self.H = np.zeros((self.nMeasurements, self.nStates))
        self.H[0, 0] = 1
        self.H[1, 3] = 1
        return

    def initial_covariance_matrix(self):
        return np.eye(self.nStates) * 500.0

    def process_noise_matrix(self, std_acceleration: float):
        Q = np.ones((self.nStates, self.nStates))
        # Assuming process noise in X and Y is uncorrelated
        Q[:3, 3:] = 0.0
        Q[3:, :3] = 0.0

        Q[0, 0] = 0.25 * self.dt**4.0
        Q[0, 1] = 0.5 * self.dt**3.0
        Q[0, 2] = 0.5 * self.dt**2.0
        Q[1, 0] = 0.5 * self.dt**3.0
        Q[1, 1] = self.dt**2.0
        Q[1, 2] = self.dt
        Q[2, 0] = 0.5 * self.dt**2.0
        Q[2, 1] = self.dt

        Q[3, 3] = 0.25 * self.dt**4.0
        Q[3, 4] = 0.5 * self.dt**3.0
        Q[3, 5] = 0.5 * self.dt**2.0
        Q[4, 3] = 0.5 * self.dt**3.0
        Q[4, 4] = self.dt**2.0
        Q[4, 5] = self.dt
        Q[5, 3] = 0.5 * self.dt**2.0
        Q[5, 4] = self.dt

        # Get tandom variance in acceleration
        cov_a = Covariance(std=std_acceleration)
        return Q * cov_a.variance

    def state_transition_matrix(self):
        F = np.eye(self.nStates)
        # Position x
        F[0, 1] = self.dt
        F[0, 2] = 0.5 * self.dt**2.0
        # Speed x
        F[1, 2] = self.dt

        # Position y
        F[3, 4] = self.dt
        F[3, 5] = 0.5 * self.dt**2.0
        # Speed y
        F[4, 5] = self.dt
        return F

    def load_measurement(self, x, y):
        self.z = np.zeros((2, 1))
        self.z[0] = x
        self.z[1] = y
        return

    def predict(self):
        # State
        self.predicted_state_vector = self.F @ self.state_vector + self.w
        # uncertainty
        self.P_pred = self.F @ self.P @ (self.F.T) + self.Q

    def update(self):
        innovation_state = self.z - self.H @ self.predicted_state_vector

        transposed_H = self.H.T
        innovation_covariance = self.H @ self.P_pred @ transposed_H + self.R

        kalman_gain = (self.P_pred @ transposed_H) @ np.linalg.pinv(
            innovation_covariance
        )
        self.K = kalman_gain

        self.state_vector = self.predicted_state_vector + kalman_gain @ innovation_state
        mat = np.eye(self.nStates) - kalman_gain @ self.H
        self.P = (mat @ self.P_pred) @ mat.T + kalman_gain @ self.R @ kalman_gain.T
        return
