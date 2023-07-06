from mpu6050 import mpu6050
import numpy as np

# Initialize the MPU6050 sensor
sensor = mpu6050(0x68)

# Define the Kalman filter parameters
Q_angle = 0.001
Q_gyro = 0.005
R_angle = 0.5

# Define the initial state and covariance
x = np.array([[0], [0]])  # angle, gyro bias
P = np.array([[0.1, 0], [0, 0.1]])

# Define the last time and last angle for the Kalman filter
last_time = None
last_angle = 0

# Define the function to get the current angle using the MPU6050 and Kalman filter
def get_current_angle():
    global x, P, last_time, last_angle
    
    # Get the raw sensor data from the MPU6050
    accel_data = sensor.get_accel_data()
    gyro_data = sensor.get_gyro_data()
    
    # Calculate the current time and time delta since the last measurement
    current_time = sensor.get_time()
    delta_t = (current_time - last_time) / 1000.0 if last_time is not None else 0.0
    last_time = current_time
    
    # Calculate the angle from the accelerometer data
    angle_acc = np.arctan2(accel_data['y'], np.sqrt(accel_data['x'] ** 2 + accel_data['z'] ** 2)) * 180.0 / np.pi
    
    # Calculate the rate of change of angle from the gyro data
    rate_gyro = gyro_data['x'] - x[1]
    
    # Calculate the predicted angle and gyro bias using the Kalman filter
    angle_pred = last_angle + rate_gyro * delta_t
    x_pred = np.array([[angle_pred], [x[1]]])
    P_pred = P + np.array([[delta_t * (P[1,1] - Q_gyro), -delta_t * P[1,1]], [-delta_t * P[1,1], delta_t * Q_gyro]])
    
    # Calculate the Kalman gain
    K = P_pred / (P_pred[0,0] + R_angle)
    
    # Calculate the updated state and covariance using the Kalman filter
    z = np.array([[angle_acc]])
    y = z - x_pred[0,0]
    x = x_pred + K.dot(y)
    P = (np.eye(2) - K).dot(P_pred)
    
    # Set the last angle to the current predicted angle
    last_angle = x[0,0]
    
    # Return the current angle from the Kalman filter
    return last_angle


print(get_current_angle())