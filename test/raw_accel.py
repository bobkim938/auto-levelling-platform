from mpu6050 import mpu6050
import time
import matplotlib.pyplot as plt
import numpy as np


# Initialize the MPU6050 sensor
sensor = mpu6050(0x68)

#Get Raw gyro data from MPU6050
current_time = time.time()
test_time = 30
accelx = []
accely = []
accelz = []

end_test = current_time + test_time
t = []



t0 = 0

while end_test > time.time():
    accel_data = sensor.get_accel_data()
    accel_x = accel_data['x']
    accel_y = accel_data['y']
    accel_z = accel_data['z']
    accelx.append(accel_x)
    accely.append(accel_y)
    accelz.append(accel_z)
    t0 += (6.38*(10**-3))
    t.append(t0)

plt.plot(t, accelx, label='Raw Accelerometer x')
plt.plot(t, accely, label='Raw Accelerometer y')
plt.plot(t, accelz, label='Raw Accelerometer z')
plt.legend(['x', 'y', 'z'])
plt.xlabel('Time (s)')
plt.ylabel('m/s^2')
plt.show()