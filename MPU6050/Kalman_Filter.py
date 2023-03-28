import numpy as np

class Kalman_Filter:
    def __init__(self):
        #Covariance of process noise
        self.Q_theta = 0.001  
        self.Q_gy_bias = 0.003  

        #Covariance of measurement noise
        self.R_angle = 0.03  

        self.x = np.array([[0.0], [0.0]])  # angle, gyro bias
        self.P = np.array([[0.0, 0.0], [0.0, 0.0]]) #priori convariance matrix

        self.angle = 0.0 #initial angle set at 0
        self.bias = 0.0 #initial gyro bias set at 0

    def get_angle(self, angle_new, gyro_rate, dt):
        #Priori State Calculation
        self.rate = gyro_rate - self.bias
        self.angle += dt * self.rate

        #Setting up priori covariance matrix
        self.P[0][0] -= (self.P[1][0]+self.P[0][1]-self.P[1][1]*dt-self.Q_theta)*dt
        self.P[0][1] -= self.P[1][1]*dt
        self.P[1][0] -= self.P[1][1]*dt
        self.P[1][1] += self.Q_gy_bias*dt

        #Calculate difference between the actual measurement and the predicted measurement based on the current state estimate
        y = angle_new - self.angle

        #Set innovation covariance(S) to the priori covariance(P) plus the measurement noise covariance(R)
        S = self.P[0][0] + self.R_angle
        
        #Calculate Kalman Gain, indication of how much the current state estimate is corrected by the measurement
        K = np.array([self.P[0][0]/S, self.P[1][0]/S]) 

        #Update the state estimate
        self.bias += K[1]*y
        self.angle += K[0]*y

        #Update posteriori error covariance matrix
        temp00 = self.P[0][0]
        temp01 = self.P[0][1]
        self.P[0][0] -= K[0]*temp00
        self.P[0][1] -= K[0]*temp01
        self.P[1][0] -= K[1]*temp00
        self.P[1][1] -= K[1]*temp01

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

    def get_Q_angle(self):
        return self.Q_angle

    def get_Q_gy_bias(self):
        return self.Q_gy_bias

    def get_R_angle(self):
        return self.R_angle










