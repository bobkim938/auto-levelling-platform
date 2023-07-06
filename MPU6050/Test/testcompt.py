from mpu6050 import mpu6050
import math
import time

# Initialize the sensor
sensor = mpu6050(0x68)


alpha = 0.98

# Initialize the variables
angle_x = 0
angle_y = 0
angle_z = 0


# Define the complementary filter function
def comp_filter(acc_data, gyro_data, alpha):
    # Calculate the accelerometer angles
    acc_x = math.atan2(acc_data['y'], acc_data['z']) * 180 / math.pi
    acc_y = math.atan2(-acc_data['x'], math.sqrt(acc_data['y'] ** 2 + acc_data['z'] ** 2)) * 180 / math.pi

    # Calculate the gyro angles
    gyro_x = gyro_data['x'] * dt
    gyro_y = gyro_data['y'] * dt
    gyro_z = gyro_data['z'] * dt

    # Calculate the complementary filter angles
    angle_x = alpha * (angle_x + gyro_x) + (1 - alpha) * acc_x
    angle_y = alpha * (angle_y + gyro_y) + (1 - alpha) * acc_y
    angle_z = gyro_z

    return angle_x, angle_y, angle_z


# Main loop
while True:
    # Read the sensor data
    st = time.time()
    accel_data = sensor.get_accel_data()
    gyro_data = sensor.get_gyro_data()
    et = time.time() - st
    # Apply the complementary filter
    angle_x, angle_y, angle_z = comp_filter(accel_data, gyro_data, et)

    # Print the angles
    print("X angle: ", angle_x)
    print("Y angle: ", angle_y)
    print("Z angle: ", angle_z)
