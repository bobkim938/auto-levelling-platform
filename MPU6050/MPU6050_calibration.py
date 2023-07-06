from mpu6050 import mpu6050
import time
import numpy as np
from Kalman_Filter import Kalman_Filter

feedback = mpu6050(0x68) # initialize MPU6050 module

kalman_filter_x = Kalman_Filter() # Kalman_Filter Initialization (x-axis)
kalman_filter_y = Kalman_Filter() # Kalman_Filter Initialization (y-axis)

# calibration step
gyro_bias_x = 0.0
for i in range(100):
    gyro_data = feedback.get_gyro_data()
    gyro_bias_x += gyro_data['x']
gyro_bias_x /= 100
kalman_filter_x.bias = gyro_bias_x

gyro_bias_y = 0.0
for i in range(100):
    gyro_data = feedback.get_gyro_data()
    gyro_bias_y += gyro_data['y']
gyro_bias_y /= 100
kalman_filter_y.bias = gyro_bias_y

prev_time = time.time()

while 1:
    # read raw data from MPU6050 module
    accel_data = feedback.get_accel_data()
    gyro_data = feedback.get_gyro_data()

    # calculate angle from accelerometer data
    accel_x = accel_data['x']
    accel_y = accel_data['y']
    accel_z = accel_data['z']

    angle_new_x = np.arctan2(accel_x, np.sqrt(accel_y**2 + accel_z**2)) * 180/np.pi
    angle_new_y = np.arctan2(accel_y, np.sqrt(accel_x**2 + accel_z**2)) * 180/np.pi


    # obtain raw angular rate from gyroscope data
    gyro_x = gyro_data['x']
    gyro_y = gyro_data['y']
    gyro_z = gyro_data['z']
    gyro_rate_x = gyro_x
    gyro_rate_y = gyro_y

    # calculate time interval between measurements
    current_time = time.time()
    dt = current_time - prev_time
    prev_time = current_time

    # obtain angle estimate using Kalman Filter
    angle_estimate_x = kalman_filter_x.get_angle(angle_new_x, gyro_rate_x, dt)
    angle_estimate_y = kalman_filter_y.get_angle(angle_new_y, gyro_rate_y, dt)

    
    # do something with the angle estimate (e.g. print to console)
    print("x:", angle_estimate_x)
    print("y:", angle_estimate_y)
    time.sleep(2)
    
