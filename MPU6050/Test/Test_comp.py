from mpu6050 import mpu6050
import time
import matplotlib.pyplot as plt
import numpy as np
from math import sin, cos, tan, pi


# Initialize the MPU6050 sensor
sensor = mpu6050(0x68)
test_time = 30
c_roll = []
c_pitch = []
t = []

end_test = time.time() + test_time

a = 0.98
gr = 0    #gyro roll
gp = 0    #gyro pitch
t0 = 0
phi_hat = 0.0
theta_hat = 0.0


while end_test > time.time():
    past_time = time.time()
    gyro_data = sensor.get_gyro_data()
    accel_data = sensor.get_accel_data()

    #obtaining gyro-rates
    gyro_x = gyro_data['x']
    gyro_y = gyro_data['y']
    gyro_z = gyro_data['z']

    #obtaining linear acceleration
    accel_x = accel_data['x']
    accel_y = accel_data['y']
    accel_z = accel_data['z']
    r = np.arctan2(accel_y, accel_z) * -1
    p = np.arctan2(accel_x, np.sqrt(accel_y ** 2 + accel_z ** 2))

    #Converting Body rates to Euler rates
    phi_dot = gyro_x + sin(phi_hat) * tan(theta_hat) * gyro_y + cos(phi_hat) * tan(theta_hat) * gyro_z
    theta_dot = cos(phi_hat) * gyro_y - sin(phi_hat) * gyro_z

    #update complementary angles
    current_time = time.time()
    dt = current_time - past_time
    phi_hat = a * (phi_hat + phi_dot * dt) + (1 - a) * r
    theta_hat = a * (theta_hat + theta_dot * dt) + (1 - a) * p
    phi_hat_deg = phi_hat * (180 / pi)
    theta_hat_deg = theta_hat * (180 / pi)
    c_roll.append(phi_hat_deg)
    c_pitch.append(theta_hat_deg)
    t0 += (0.01245)
    t.append(t0)

plt.plot(t, c_roll, label='Complementary Filtered Roll')
plt.plot(t, c_pitch, label='Complementary Filtered Pitch')
plt.legend(['Comp Roll', 'Comp Pitch'])
plt.xlabel('Time (s)')
plt.ylabel('Angle (deg)')
plt.show()


