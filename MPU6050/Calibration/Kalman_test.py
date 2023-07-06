from Kalman_Filter import KalmanFilter
import numpy as np
import matplotlib.pyplot as plt
import time
from mpu6050 import mpu6050

kfr = KalmanFilter()
kfp = KalmanFilter()

test_time = 30
end_test = time.time() + test_time
t = []
filtered_x = []
filtered_y = []
t0 = 0

sensor = mpu6050(0x68)
def get_roll():
    for i in range(5):
        prev_time = time.time()
        gyro_data = sensor.get_gyro_data()
        accel_data = sensor.get_accel_data()
        gyro_x = gyro_data['x'] - (-1.4764528604678508)
        acc_x_off = accel_data['x'] * (-0.49996226952053047) + 0.06969925934469653
        acc_y_off = accel_data['y'] * (-0.499933868982771) + (-0.06087691361286952)
        acc_z_off = accel_data['z'] * (-0.4933871810249762) + 1.7733706385934813
        accel_x = accel_data['x'] - acc_x_off
        accel_y = accel_data['y'] - acc_y_off
        accel_z = accel_data['z'] - acc_z_off
        roll = -1 * np.arctan2(accel_y, np.sqrt(accel_x ** 2 + accel_z ** 2)) * (180 / np.pi)
        dt = time.time() - prev_time
        r = kfr.get_angle(roll, gyro_x, dt)
        roll += r
    roll = roll / 5
    return roll


def get_pitch():
    for i in range(5):
        prev_time = time.time()
        gyro_data = sensor.get_gyro_data()
        gyro_y = gyro_data['y'] - 0.804882200137513
        accel_data = sensor.get_accel_data()
        acc_x_off = accel_data['x'] * (-0.49996226952053047) + 0.06969925934469653
        acc_y_off = accel_data['y'] * (-0.499933868982771) + (-0.06087691361286952)
        acc_z_off = accel_data['z'] * (-0.4933871810249762) + 1.7733706385934813
        accel_x = accel_data['x'] - acc_x_off
        accel_y = accel_data['y'] - acc_y_off
        accel_z = accel_data['z'] - acc_z_off
        pitch = -1 * np.arctan2(-accel_x, np.sqrt(accel_y ** 2 + accel_z ** 2)) * (180 / np.pi)
        dt = time.time() - prev_time
        p = kfp.get_angle(pitch, gyro_y, dt)
        pitch += p
    pitch = pitch / 5
    return pitch


while end_test > time.time():
    roll = get_roll()
    pitch = get_pitch()
    filtered_x.append(roll)
    filtered_y.append(pitch+0.5)
    t0 += 0.01371
    t.append(t0)

plt.plot(t, filtered_x, label='Kalman Filtered Roll')
plt.plot(t, filtered_y, label='Kalman Filtered Pitch')
plt.legend(['Kalman_roll', 'Kalman_pitch'])
plt.xlabel('Time (s)')
plt.ylabel('Angle (deg)')
plt.show()
