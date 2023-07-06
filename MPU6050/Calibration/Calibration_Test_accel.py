import time
import matplotlib.pyplot as plt
from mpu6050 import mpu6050

test_time = 30
end_test = time.time() + test_time
t = []
t0 = 0
accelx = []
accely = []
accelz = []

sensor = mpu6050(0x68)
while end_test > time.time():
    # accel data from MPU6050 after calibration
    st = time.time()
    accel_data = sensor.get_accel_data()
    acc_x_off = accel_data['x'] * (-0.49996226952053047) + 0.06969925934469653
    acc_y_off = accel_data['y'] * (-0.499933868982771) + (-0.06087691361286952)
    acc_z_off = accel_data['z'] * (-0.4933871810249762) + 1.7733706385934813
    accel_x = accel_data['x'] - acc_x_off
    accel_y = accel_data['y'] - acc_y_off
    accel_z = accel_data['z'] - acc_z_off
    accelx.append(accel_x)
    accely.append(accel_y)
    accelz.append(accel_z)
    t0 += time.time() - st
    t.append(t0)

plt.plot(t, accelx, label='Calibrated Accel x')
plt.plot(t, accely, label='Calibrated Accel y')
plt.plot(t, accelz, label='Calibrated Accel z')
plt.ylim(-5, 10)
plt.legend(['x', 'y', 'z'])
plt.xlabel('Time (s)')
plt.ylabel('Acceleration (m/s^2)')
plt.show()
