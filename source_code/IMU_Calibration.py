from mpu6050 import mpu6050
import time


class CalibrationIMU:
    def __init__(self, address):
        self.sensor = mpu6050(address)
        self.gyro_x_offset = 0
        self.gyro_y_offset = 0
        self.gyro_z_offset = 0
        self.accel_x_m = 0
        self.accel_x_b = 0
        self.accel_y_m = 0
        self.accel_y_b = 0
        self.accel_z_m = 0
        self.accel_z_b = 0

    def calibrate_gyro(self, testing_time=10):
        gyro_x = 0
        gyro_y = 0
        gyro_z = 0
        counter = 0
        end_test = time.time() + testing_time
        while end_test > time.time():
            gyro_data = self.sensor.get_gyro_data()
            gyro_x += gyro_data['x']
            gyro_y += gyro_data['y']
            gyro_z += gyro_data['z']
            counter += 1
        self.gyro_x_offset = gyro_x / counter
        self.gyro_y_offset = gyro_y / counter
        self.gyro_z_offset = gyro_z / counter

        return self.gyro_x_offset, self.gyro_y_offset, self.gyro_z_offset

    def get_accel_data(self):
        accel_data = self.sensor.get_accel_data()
        accel_x = accel_data['x']
        accel_y = accel_data['y']
        accel_z = accel_data['z']
        return accel_x, accel_y, accel_z

    def calibrate_accel_z(self, calibration_time=10):
        # least square method for linear regression
        sample_num = 0
        x_sum = 0
        y_sum = 0
        x_squared_sum = 0
        xy_sum = 0

        print('Place the z-axis upwards, Enter any key to continue')
        input()
        end_loop_time = calibration_time + time.time()
        print('Calibration start')
        while end_loop_time > time.time():
            sample_num += 1
            offset = self.get_accel_data()[2] - 9.81

            x_sum += 9.81
            y_sum += offset
            x_squared_sum += (9.81 * 9.81)
            xy_sum += 9.81 * offset

        print('Place the z-axis perpendicular to gravity, Enter any key to continue')
        input()
        end_loop_time = calibration_time + time.time()
        print('Calibration start')
        while end_loop_time > time.time():
            sample_num += 1
            offset = self.get_accel_data()[2]
            x_sum += 0
            y_sum += offset
            x_squared_sum += 0
            xy_sum += (0 * offset)

        print('Place the z-axis downwards, Enter any key to continue')
        input()
        end_loop_time = calibration_time + time.time()
        print('Calibration start')
        while end_loop_time > time.time():
            sample_num += 1
            offset = self.get_accel_data()[2] + 9.81
            x_sum -= 9.81
            y_sum += offset
            x_squared_sum += (9.81 * 9.81)
            xy_sum += (-9.81 * offset)

        self.accel_z_m = (sample_num * xy_sum - x_sum * y_sum) / (sample_num * x_squared_sum - x_sum * x_sum)
        self.accel_z_b = (y_sum - self.accel_z_m * x_sum) / sample_num
        return self.accel_z_m, self.accel_z_b

    def calibrate_accel_x(self, calibration_time=10):
        # least square method for linear regression
        sample_num = 0
        x_sum = 0
        y_sum = 0
        x_squared_sum = 0
        xy_sum = 0

        print('Place the x-axis upwards, Enter any key to continue')
        input()
        end_loop_time = calibration_time + time.time()
        print('Calibration start')
        while end_loop_time > time.time():
            sample_num += 1
            offset = self.get_accel_data()[0] - 9.81

            x_sum += 9.81
            y_sum += offset
            x_squared_sum += (9.81 * 9.81)
            xy_sum += 9.81 * offset

        print('Place the x-axis perpendicular to gravity, Enter any key to continue')
        input()
        end_loop_time = calibration_time + time.time()
        print('Calibration start')
        while end_loop_time > time.time():
            sample_num += 1
            offset = self.get_accel_data()[0]
            x_sum += 0
            y_sum += offset
            x_squared_sum += 0
            xy_sum += (0 * offset)

        print('Place the x-axis downwards, Enter any key to continue')
        input()
        end_loop_time = calibration_time + time.time()
        print('Calibration start')
        while end_loop_time > time.time():
            sample_num += 1
            offset = self.get_accel_data()[0] + 9.81
            x_sum -= 9.81
            y_sum += offset
            x_squared_sum += (9.81 * 9.81)
            xy_sum += (-9.81 * offset)

        self.accel_x_m = (sample_num * xy_sum - x_sum * y_sum) / (sample_num * x_squared_sum - x_sum * x_sum)
        self.accel_x_b = (y_sum - self.accel_x_m * x_sum) / sample_num
        return self.accel_x_m, self.accel_x_b

    def calibrate_accel_y(self, calibration_time=10):
        # least square method for linear regression
        sample_num = 0
        x_sum = 0
        y_sum = 0
        x_squared_sum = 0
        xy_sum = 0

        print('Place the y-axis upwards, Enter any key to continue')
        input()
        end_loop_time = calibration_time + time.time()
        print('Calibration start')
        while end_loop_time > time.time():
            sample_num += 1
            offset = self.get_accel_data()[1] - 9.81

            x_sum += 9.81
            y_sum += offset
            x_squared_sum += (9.81 * 9.81)
            xy_sum += 9.81 * offset

        print('Place the y-axis perpendicular to gravity, Enter any key to continue')
        input()
        end_loop_time = calibration_time + time.time()
        print('Calibration start')
        while end_loop_time > time.time():
            sample_num += 1
            offset = self.get_accel_data()[1]
            x_sum += 0
            y_sum += offset
            x_squared_sum += 0
            xy_sum += (0 * offset)

        print('Place the y-axis downwards, Enter any key to continue')
        input()
        end_loop_time = calibration_time + time.time()
        print('Calibration start')
        while end_loop_time > time.time():
            sample_num += 1
            offset = self.get_accel_data()[1] + 9.81
            x_sum -= 9.81
            y_sum += offset
            x_squared_sum += (9.81 * 9.81)
            xy_sum += (-9.81 * offset)

        self.accel_y_m = (sample_num * xy_sum - x_sum * y_sum) / (sample_num * x_squared_sum - x_sum * x_sum)
        self.accel_y_b = (y_sum - self.accel_y_m * x_sum) / sample_num
        return self.accel_y_m, self.accel_y_b
