from Kalman_Filter import Kalman_Filter
import numpy as np
import matplotlib.pyplot as plt
import time
from mpu6050 import mpu6050

kfr = Kalman_Filter()
kfy = Kalman_Filter()
sensor = mpu6050(0x68)

# Define variables for storing the time, raw accelerometer and gyroscope data, and Kalman filter estimates
test_time = 30
t = []
filtered_roll = []
filtered_yaw = []

t0 = 0

end_test = time.time() + test_time

while end_test > time.time():
    gyro_data = sensor.get_gyro_data()
    accel_data = sensor.get_accel_data()
    prev_time = time.time()
    gyro_x = gyro_data['x']
    gyro_y = gyro_data['y']
    accel_x = accel_data['x']
    accel_y = accel_data['y']
    accel_z = accel_data['z']

    # Calculate roll angle
    r = np.arctan2(accel_y, np.sqrt(accel_x ** 2 + accel_z ** 2)) * (180 / np.pi)

    # Calculate pitch angle
    p = -1*np.arctan2(-accel_x, np.sqrt(accel_y ** 2 + accel_z ** 2)) * (180 / np.pi)

    current_time = time.time()
    dt = current_time - prev_time
    filtered_roll.append(kfr.get_angle(r, gyro_x, dt))
    filtered_yaw.append(kfy.get_angle(p, gyro_y, dt))
    t0 += (6.38 * (10 ** -3))
    t.append(t0)

plt.plot(t, filtered_roll, label='Kalman Filtered Roll')
plt.plot(t, filtered_yaw, label='Kalman Filtered Yaw')
plt.legend(['Kalman_roll', 'Kalman_yaw'])
plt.xlabel('Time (s)')
plt.ylabel('Angle (deg)')
plt.show()

