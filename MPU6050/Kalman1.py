import numpy as np
class Kalman_Filter:
    def __init__(self, Q_angle=0.001, Q_gyro=0.003, R_angle=0.03):
        self.Q_angle = Q_angle
        self.Q_gyro = Q_gyro
        self.R_angle = R_angle
        self.angle = 0.0
        self.bias = 0.0
        self.P = np.zeros((2, 2))
        self.K = np.zeros((2, 1))

    def get_angle(self, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, dt):
        # calculate angle from accelerometer data
        accel_angle_x = np.arctan2(accel_x, np.sqrt(accel_y**2 + accel_z**2))
        accel_angle_y = np.arctan2(accel_y, np.sqrt(accel_x**2 + accel_z**2))

        # obtain raw angular rate from gyroscope data
        gyro_rate_x = gyro_x - self.bias

        # update angle estimate using gyroscope data
        self.angle += dt * gyro_rate_x

        # update error covariance matrix
        self.P[0, 0] += dt * (dt*self.P[1, 1] - self.P[0, 1] - self.P[1, 0] + self.Q_angle)
        self.P[0, 1] -= dt * self.P[1, 1]
        self.P[1, 0] -= dt * self.P[1, 1]
        self.P[1, 1] += self.Q_gyro * dt

        # calculate Kalman gain
        S = self.P[0, 0] + self.R_angle
        self.K[0] = self.P[0, 0] / S
        self.K[1] = self.P[1, 0] / S

        # update angle estimate using accelerometer data
        y = np.array([[accel_angle_x], [accel_angle_y]])
        x = np.array([[self.angle], [gyro_rate_x]])
        self.angle += np.dot(self.K, (y - x))[0]

        # update error covariance matrix
        self.P -= np.dot(self.K, np.dot(self.P, np.array([[1, 0], [0, 0]])))

        # update bias estimate using gyroscope data
        self.bias += self.K[1]

        return self.angle
