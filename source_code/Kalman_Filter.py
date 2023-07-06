import numpy as np


class KalmanFilter:
    def __init__(self):
        # Covariance of process noise
        self.Q_theta = 0.001  
        self.Q_gy_bias = 0.003  

        # Covariance of measurement noise
        self.R_angle = 0.03

        self.P = np.array([[0.0, 0.0], [0.0, 0.0]])  # error covariance matrix

        self.angle = 0.0  # initial angle set at 0
        self.bias = 0.0  # initial gyro bias set at 0

        self.rate = 0.0  # initial angular rate set at 0

    def angle_estimate(self, angle_new, gyro_rate, dt):
        # initial state calculation
        self.rate = gyro_rate - self.bias
        self.angle += dt * self.rate

        # error covariance matrix estimation
        self.P[0][0] -= (self.P[1][0]+self.P[0][1]-self.P[1][1]*dt-self.Q_theta)*dt
        self.P[0][1] -= self.P[1][1]*dt
        self.P[1][0] -= self.P[1][1]*dt
        self.P[1][1] += self.Q_gy_bias*dt

        # Calculate difference between actual measurement and predicted measurement
        y = angle_new - self.angle

        # Set innovation covariance(s) to the sum of the error covariance matrix and the measurement noise covariance
        s = self.P[0][0] + self.R_angle
        
        # Calculate Kalman Gain, indication certainty of the measurement
        k = np.array([self.P[0][0]/s, self.P[1][0]/s])

        # state estimate update
        self.bias += k[1]*y
        self.angle += k[0]*y

        # update error covariance matrix
        temp00 = self.P[0][0]
        temp01 = self.P[0][1]
        self.P[0][0] -= k[0]*temp00
        self.P[0][1] -= k[0]*temp01
        self.P[1][0] -= k[1]*temp00
        self.P[1][1] -= k[1]*temp01

        return self.angle
    
    def set_angle(self, angle):
        self.angle = angle

    def get_rate(self):
        return self.rate

    def set_Q_angle(self, Q_angle):
        self.Q_angle = Q_angle

    def set_Q_gy_bias(self, Q_gy_bias):
        self.Q_gy_bias = Q_gy_bias

    def set_R_angle(self, R_angle):
        self.R_angle = R_angle












