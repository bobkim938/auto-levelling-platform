import matplotlib.pyplot as plt
from Kalman_Filter import Kalman_Filter
import numpy as np
import time
from mpu6050 import mpu6050

# Create an instance of the Kalman filter
kf = Kalman_Filter()

sensor = mpu6050(0x68)

# Define variables for storing the time, raw accelerometer and gyroscope data, and Kalman filter estimates
t = []
raw_accel_x = []
raw_gyro_x = []
kf_angle = []
Comp_angle = []


gyro_bias_x = 0.0
for i in range(100):
    gyro_data = sensor.get_gyro_data()
    gyro_bias_x += gyro_data['x']
gyro_bias_x /= 100
kf.bias = gyro_bias_x

last_time = time.time()
angle_x_comp_filtered = 0.0
gyro_angle_raw = 0.0

for i in range(1000):
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

    gyro_angle_raw += rate_x * dt
    
    # Filter the roll and pitch angles using the Kalman filter
    angle_x_kalman_filtered = kf.get_angle(angle_x, rate_x, dt)
    angle_x_comp_filtered = 0.98 * (rate_x * dt) + 0.02 * angle_x
    
    # Append the data to the respective lists
    t.append(last_time - t[0] if len(t) > 0 else 0)  # subtract the initial time to start from zero seconds
    raw_accel_x.append(angle_x)
    raw_gyro_x.append(gyro_angle_raw)
    Comp_angle.append(angle_x_comp_filtered)
    kf_angle.append(angle_x_kalman_filtered)



# Plot the data
plt.figure()
plt.plot(t, raw_accel_x, label='Raw Accelerometer Roll')
plt.plot(t, raw_gyro_x, label='Raw Gyroscope Roll')
plt.plot(t, Comp_angle, label='Complementary Filtered Angle Roll')
plt.plot(t, kf_angle, label='Kalman Filtered Angle Roll')
plt.xlabel('Time (s)')
plt.ylabel('Angle (degrees)')
plt.legend(['Accel_Roll', 'Gyro_Roll', 'Complementary_Roll', 'kalman_x'])
plt.show()