from mpu6050 import mpu6050
import time
import matplotlib.pyplot as plt

# Initialize the MPU6050 sensor
sensor = mpu6050(0x68)

#Get Raw gyro data from MPU6050
current_time = time.time()
test_time = 30
rate_x = []
rate_y = []
rate_z = []
end_test = current_time + test_time
t = []

t0 = 0

while end_test > time.time():
    gyro_data = sensor.get_gyro_data()
    rate_x.append(gyro_data['x'])
    rate_y.append(gyro_data['y'])
    rate_z.append(gyro_data['z'])
    t0 += (6.38 * (10 ** -3))
    t.append(t0)

plt.plot(t, rate_x, label='Raw Gyroscope x')
plt.plot(t, rate_y, label='Raw Gyroscope y')
plt.plot(t, rate_z, label='Raw Gyroscope z')
plt.ylim(-10, 15)
plt.legend(['x', 'y', 'z'])
plt.xlabel('Time (s)')
plt.ylabel('Angular Velocity (deg/s)')
plt.show()



