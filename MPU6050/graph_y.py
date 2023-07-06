import matplotlib.pyplot as plt
from Kalman_Filter import Kalman_Filter
import numpy as np
import time

# Create an instance of the Kalman filter
kf = Kalman_Filter()

# Define variables for storing the time, raw accelerometer and gyroscope data, and Kalman filter estimates
t = []
raw_accel_y = []
raw_gyro_y = []
kf_angle = []

# Loop through the data and filter the measurements using the Kalman filter
for i in range(len(data)):
    # Get the raw accelerometer and gyroscope measurements
    accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = data[i]
    
    # Compute the roll and pitch angles using the accelerometer data
    angle_y = np.arctan2(-accel_x, np.sqrt(accel_y**2 + accel_z**2)) * 180 / np.pi
    
    # Compute the angular rate using the gyroscope data
    rate_y = gyro_y / 131.0
    
    # Get the time elapsed since the last measurement
    dt = (time.time() - last_time)
    last_time = time.time()
    
    # Filter the roll and pitch angles using the Kalman filter
    angle_y_Kalman = kf.get_angle(angle_y, rate_y, dt)
    
    # Append the data to the respective lists
    t.append(last_time)
    raw_accel_y.append(accel_y)
    raw_gyro_y.append(rate_y)
    kf_angle.append(angle_y_Kalman)
    
# Plot the data
plt.figure()
plt.plot(t, raw_accel_y, label='Raw Accelerometer Y')
plt.plot(t, raw_gyro_y, label='Raw Gyroscope Y')
plt.plot(t, kf_angle, label='Kalman Filtered Angle Y')
plt.xlabel('Time (s)')
plt.ylabel('Angle (degrees)')
plt.legend()
plt.show()
