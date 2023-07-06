import matplotlib.pyplot as plt
from Kalman_Filter import Kalman_Filter
import numpy as np
import time
from mpu6050 import mpu6050


sensor = mpu6050(0x68)

# Define variables for storing the time, raw accelerometer and gyroscope data, and Kalman filter estimates
t = []
raw_accel_x = []
raw_gyro_x = []
raw_gyro_angle = []

last_time = time.time()

for i in range(100):
    # Get the raw accelerometer and gyroscope measurements from the MPU6050 sensor
    accel_data = sensor.get_accel_data()
    gyro_data = sensor.get_gyro_data()
    
    # Get the roll and pitch angles using the MPU6050 sensor data
    accel_x = accel_data['x']
    accel_y = accel_data['y']
    accel_z = accel_data['z']
    angle_x = np.arctan2(accel_y, np.sqrt(accel_x**2 + accel_z**2)) * 180 / np.pi
    angle_y = np.arctan2(-accel_x, np.sqrt(accel_y**2 + accel_z**2)) * 180 / np.pi
    
    # Compute the angular rate using the gyroscope data (scaling)
    rate_x = gyro_data['x'] / 131.0
    rate_y = gyro_data['y'] / 131.0
    
    # Get the time elapsed since the last measurement
    dt = (time.time() - last_time)
    last_time = time.time()

    gyro_angle_x = 0
    gyro_angle_x += rate_x * dt
    
    # Append the data to the respective lists
    t.append(last_time)  
    raw_accel_x.append(accel_x)
    raw_gyro_x.append(rate_x)
    raw_gyro_angle.append(gyro_angle_x)
    time.sleep(1)


# Plot the data
plt.figure()
plt.plot(t, raw_accel_x, label='Raw Accelerometer X')
plt.plot(t, raw_gyro_x, label='Raw Gyroscope X')
plt.plot(t, raw_gyro_angle, label='Raw Gyroscope Angle X')
plt.xlabel('Time (s)')
plt.ylabel('Angle (degrees)')
plt.legend(['accel_x', 'gyro_x', 'Raw Gyroscope Angle X'])
plt.show()