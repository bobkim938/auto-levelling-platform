from IMU_Calibration import CalibrationIMU
import time
import matplotlib.pyplot as plt
from mpu6050 import mpu6050


calibration = CalibrationIMU(0x68)
gyro_offset = calibration.calibrate_gyro()

test_time = 30
end_test = time.time() + test_time
t = []
t0 = 0
gyrox = []
gyroy = []
gyroz = []


sensor = mpu6050(0x68)
while end_test > time.time():
    #gyro data from MPU6050 after calibration
    gyro_data = sensor.get_gyro_data()
    gyrox.append((gyro_data['x'] - gyro_offset[0]))
    gyroy.append((gyro_data['y'] - gyro_offset[1]))
    gyroz.append((gyro_data['z'] - gyro_offset[2]))
    t0 += (6.479 * (10 ** -3))


plt.plot(t, gyrox, label='Calibrated gyro x')
plt.plot(t, gyroy, label='Calibrated gyro y')
plt.plot(t, gyroz, label='Calibrated gyro z')
plt.ylim(-10, 15)
plt.legend(['x', 'y', 'z'])
plt.xlabel('Time (s)')
plt.ylabel('angular velocity (deg/s)')
plt.show()