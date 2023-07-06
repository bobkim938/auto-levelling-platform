import busio
import board
from adafruit_pca9685 import PCA9685
import adafruit_motor.servo
from mpu6050 import mpu6050
import numpy as np
import time
from Kalman_Filter import KalmanFilter
from IMU_Calibration import CalibrationIMU

kfr = KalmanFilter()  # initialize Kalman Filter
kfp = KalmanFilter()
sensor = mpu6050(0x68)
i2c = busio.I2C(board.SCL, board.SDA)  # Set up the I2C bus using GPIO 2, 3
pca = PCA9685(i2c)
pca.frequency = 50  # Set the PWM frequency to 50hz
motor1 = adafruit_motor.servo.ContinuousServo(pca.channels[0])  # Set the servo to channel 0
motor2 = adafruit_motor.servo.ContinuousServo(pca.channels[2])  # Set the servo to channel 1
motor3 = adafruit_motor.servo.ContinuousServo(pca.channels[4])  # Set the servo to channel 2
kp = 0.2
ki = 0.02
kd = 0.001
pasterror = 0.0
error = 0.0
max_pid = 1
min_pid = -1
tau = 3
sample_t = 0.0
proportional = 0.0
integral = 0.0
derivative = 0.0
prevmeasurement_roll = 0.0
prevmeasurement_pitch = 0.0
pid_output = 0.0
max_rp = 5
min_rp = -5
z_off = []
x_off = []
y_off = []
gyro_off = []


def pid_roll(measurement, dt, setpoint=0):
    global kp, ki, kd
    global error, pasterror
    global max_pid, min_pid  # for integrator anti-windup
    global tau, sample_t
    global proportional, integral, derivative, prevmeasurement_roll
    global pid_output

    # Roll and pitch limits of the system
    if measurement > max_rp:
        measurement = max_rp
    elif measurement < min_rp:
        measurement = min_rp
    else:
        measurement = measurement

    error = setpoint - measurement  # set-point always 0 on both roll and pitch
    sample_t = dt
    # PID difference equations
    proportional = kp * error  # proportional

    # Calculate scaling factor based on desired output range and servo characteristics
    max_rot_speed = 78  # Maximum rotational speed of the FS5106R servo in RPM
    min_rot_speed = -78  # Minimum rotational speed of the FS5106R servo in RPM
    scaling_factor = (max_rot_speed - min_rot_speed) / (max_pid - min_pid)

    # Calculate output limits for anti-windup
    max_pid = max_rot_speed / scaling_factor
    min_pid = min_rot_speed / scaling_factor

    integral += float(((ki * sample_t) / 2) * (error + pasterror))  # integral

    # integral term anti-windup
    if integral > max_pid:
        integral = max_pid
    elif integral < min_pid:
        integral = min_pid

    derivative = ((2 * kd) / (sample_t + 2 * tau)) * (measurement - prevmeasurement_roll) - (
            (sample_t - 2 * tau) / (sample_t + 2 * tau)) * derivative  # derivative

    pid_output = proportional + integral + derivative

    # Scale PID output to servo rotational speed range
    pid_output = pid_output * scaling_factor

    # Output limits for anti-windup
    if pid_output > max_rot_speed:
        pid_output = max_rot_speed
    elif pid_output < min_rot_speed:
        pid_output = min_rot_speed

    pasterror = error
    prevmeasurement_roll = measurement
    result = abs(pid_output) / 78

    return result


def pid_pitch(measurement, dt, setpoint=0):
    global kp, ki, kd
    global error, pasterror
    global max_pid, min_pid  # for integrator anti-windup
    global tau, sample_t
    global proportional, integral, derivative, prevmeasurement_pitch
    global pid_output

    # Roll and pitch limits of the system
    if measurement > max_rp:
        measurement = max_rp
    elif measurement < min_rp:
        measurement = min_rp
    else:
        measurement = measurement

    error = setpoint - measurement  # set-point always 0 on both roll and pitch
    sample_t = dt
    # PID difference equations
    proportional = kp * error  # proportional

    # Calculate scaling factor based on desired output range and servo characteristics
    max_rot_speed = 78  # Maximum rotational speed of the FS5106R servo in RPM
    min_rot_speed = -78  # Minimum rotational speed of the FS5106R servo in RPM
    scaling_factor = (max_rot_speed - min_rot_speed) / (max_pid - min_pid)

    # Calculate output limits for anti-windup
    max_pid = max_rot_speed / scaling_factor
    min_pid = min_rot_speed / scaling_factor

    integral += float(((ki * sample_t) / 2) * (error + pasterror))  # integral

    # integral term anti-windup
    if integral > max_pid:
        integral = max_pid
    elif integral < min_pid:
        integral = min_pid

    derivative = ((2 * kd) / (sample_t + 2 * tau)) * (measurement - prevmeasurement_pitch) - (
            (sample_t - 2 * tau) / (sample_t + 2 * tau)) * derivative  # derivative

    pid_output = proportional + integral + derivative

    # Scale PID output to servo rotational speed range
    pid_output = pid_output * scaling_factor

    # Output limits for anti-windup
    if pid_output > max_rot_speed:
        pid_output = max_rot_speed
    elif pid_output < min_rot_speed:
        pid_output = min_rot_speed

    pasterror = error
    prevmeasurement_pitch = measurement
    result = abs(pid_output) / 78

    return result


def calibration_gyro(address=0x68):
    gyro_calib = CalibrationIMU(address)
    gyro_offset = gyro_calib.calibrate_gyro()
    return gyro_offset


def calibration_az(address=0x68):
    az_calib = CalibrationIMU(address)
    az_offset = az_calib.calibrate_accel_z()
    return az_offset


def calibration_ax(address=0x68):
    ax_calib = CalibrationIMU(address)
    ax_offset = ax_calib.calibrate_accel_x()
    return ax_offset


def calibration_ay(address=0x68):
    ay_calib = CalibrationIMU(address)
    ay_offset = ay_calib.calibrate_accel_y()
    return ay_offset


def get_roll():
    roll = 0
    for i in range(5):
        prev_time = time.time()
        gyro_data = sensor.get_gyro_data()
        accel_data = sensor.get_accel_data()
        gyro_x = gyro_data['x'] - gyro_off[0]
        acc_x_off = accel_data['x'] * x_off[0] + x_off[1]
        acc_y_off = accel_data['y'] * y_off[0] + y_off[1]
        acc_z_off = accel_data['z'] * z_off[0] + z_off[1]
        accel_x = accel_data['x'] - acc_x_off
        accel_y = accel_data['y'] - acc_y_off
        accel_z = accel_data['z'] - acc_z_off
        roll = -1 * np.arctan2(accel_y, np.sqrt(accel_x ** 2 + accel_z ** 2)) * (180 / np.pi)
        dt = time.time() - prev_time
        r = kfr.angle_estimate(roll, gyro_x, dt)
        roll += r
    roll = roll / 5
    return roll


def get_pitch():
    pitch = 0
    for i in range(5):
        prev_time = time.time()
        gyro_data = sensor.get_gyro_data()
        gyro_y = gyro_data['y'] - gyro_off[1]
        accel_data = sensor.get_accel_data()
        acc_x_off = accel_data['x'] * x_off[0] + x_off[1]
        acc_y_off = accel_data['y'] * y_off[0] + y_off[1]
        acc_z_off = accel_data['z'] * z_off[0] + z_off[1]
        accel_x = accel_data['x'] - acc_x_off
        accel_y = accel_data['y'] - acc_y_off
        accel_z = accel_data['z'] - acc_z_off
        pitch = -1 * np.arctan2(-accel_x, np.sqrt(accel_y ** 2 + accel_z ** 2)) * (180 / np.pi)
        dt = time.time() - prev_time
        p = kfp.angle_estimate(pitch, gyro_y, dt)
        pitch += p
    pitch = pitch / 5
    return pitch


def main():
    global z_off
    global x_off
    global y_off
    global gyro_off
    gyro_off = calibration_gyro()
    z_off = calibration_az()
    x_off = calibration_ax()
    y_off = calibration_ay()
    print("Calibration done")
    roll = 0.0
    pitch = 0.0

    # boolean variables for platform balance
    balance_roll = True
    balance_pitch = True
    # position of the servos
    pos1 = np.array([0, 127, 127 * np.tan(roll * np.pi / 180)])
    pos2 = np.array([110, -63.5, 110 * np.tan(pitch * np.pi / 180)])
    pos3 = np.array([-110, -63.5, 110 * np.tan(pitch * np.pi / 180)])
    end_time = time.time() + 5
    while end_time > time.time():
        get_roll()
        get_pitch()

    try:
        while True:
            roll = int(get_roll())
            pitch = int(get_pitch())

            if roll > 0 or roll < 0:
                balance_roll = False

            if pitch > 0 or pitch < 0:
                balance_pitch = False

            if balance_roll and balance_pitch:
                motor1.throttle = 0
                motor2.throttle = 0
                motor3.throttle = 0

            if not balance_roll:
                while not balance_roll:
                    print("adjusting roll")
                    start_time = time.time()
                    roll = int(get_roll())
                    roll_time = time.time()
                    if roll > 0.0:
                        print("roll is positive", roll)
                        if pos2[2] >= abs((127 * np.tan(roll * np.pi / 180))) and pos3[2] >= abs((
                                127 * np.tan(roll * np.pi / 180))):
                            motor_constant = -1
                            dt = time.time() - start_time
                            motor1.throttle = 0
                            motor2.throttle = motor_constant * pid_roll(roll, dt)
                            motor3.throttle = motor_constant * pid_roll(roll, dt)
                            diff_roll = get_roll() - roll
                            pos2[2] -= 127 * abs(np.tan(diff_roll * np.pi / 180))
                            pos3[2] -= 127 * abs(np.tan(diff_roll * np.pi / 180))
                            print("motor2 and motor3 are on (CW)")
                            print("pos2:", pos2[2], " pos3:", pos3[2])
                        else:
                            motor_constant = 1
                            dt = time.time() - start_time
                            motor1.throttle = motor_constant * pid_roll(roll, dt)
                            motor2.throttle = 0
                            motor3.throttle = 0
                            diff_roll = get_roll() - roll
                            pos1[2] += 127 * abs(np.tan(diff_roll * np.pi / 180))
                            print("motor1 is on (CCW)")
                            print("pos1:", pos1[2])
                    elif roll < 0.0:
                        print("roll is negative", roll)
                        if pos1[2] >= abs((127 * np.tan(roll * np.pi / 180))):
                            motor_constant = -1
                            dt = time.time() - start_time
                            motor1.throttle = motor_constant * pid_roll(roll, dt)
                            motor2.throttle = 0
                            motor3.throttle = 0
                            diff_roll = get_roll() - roll
                            pos1[2] -= 127 * abs(np.tan(diff_roll * np.pi / 180))
                            print("motor1 is on (CW)")
                            print("pos1:", pos1[2])
                        else:
                            motor_constant = 1
                            dt = time.time() - start_time
                            pid_r = pid_roll(roll, dt)
                            motor1.throttle = 0
                            motor2.throttle = motor_constant * pid_r
                            motor3.throttle = motor_constant * pid_r
                            diff_roll = get_roll() - roll
                            pos2[2] += 127 * abs(np.tan(diff_roll * np.pi / 180))
                            pos3[2] += 127 * abs(np.tan(diff_roll * np.pi / 180))
                            print("motor2 and motor3 are on (CCW)")
                            print("pos2:", pos2[2], " pos3:", pos3[2])

                    if roll == 0.0:
                        print("roll adjusted")
                        print("pos1, ", pos1[2], " pos2, ", pos2[2], " pos3, ", pos3[2])
                        roll_end = time.time() - roll_time
                        print("adjusted in:", roll_end, " seconds")
                        motor1.throttle = 0
                        motor2.throttle = 0
                        motor3.throttle = 0
                        balance_roll = True

            if not balance_pitch:
                while not balance_pitch:
                    print("adjusting pitch", pitch)
                    start_time = time.time()
                    pitch = int(get_pitch())
                    pitch_time = time.time()
                    if pitch > 0.0:
                        print("pitch is positive")
                        if pos2[2] >= abs((110 * np.tan(pitch * np.pi / 180))):
                            motor_constant = -1
                            dt = time.time() - start_time
                            motor1.throttle = 0
                            motor2.throttle = motor_constant * pid_pitch(pitch, dt)
                            motor3.throttle = 0
                            diff_pitch = get_pitch() - pitch
                            pos2[2] -= 110 * abs(np.tan(diff_pitch * np.pi / 180))
                            print("motor2 is on (CW)")
                            print("pos2:", pos2[2])
                        else:
                            motor_constant = 1
                            dt = time.time() - start_time
                            motor1.throttle = 0
                            motor2.throttle = 0
                            motor3.throttle = motor_constant * pid_pitch(pitch, dt)
                            diff_pitch = get_pitch() - pitch
                            pos3[2] += 110 * abs(np.tan(diff_pitch * np.pi / 180))
                            print("motor3 is on (CCW)")
                            print("pos3:", pos3[2])
                    elif pitch < 0.0:
                        print("pitch is negative", pitch)
                        if pos3[2] >= abs((110 * np.tan(pitch * np.pi / 180))):
                            motor_constant = -1
                            dt = time.time() - start_time
                            motor1.throttle = 0
                            motor2.throttle = 0
                            motor3.throttle = motor_constant * pid_pitch(pitch, dt)
                            diff_pitch = get_pitch() - pitch
                            pos3[2] -= 110 * abs(np.tan(diff_pitch * np.pi / 180))
                            print("motor3 is on (CW)")
                            print("pos3:", pos3[2])
                        else:
                            motor_constant = 1
                            dt = time.time() - start_time
                            motor1.throttle = 0
                            motor2.throttle = motor_constant * pid_pitch(pitch, dt)
                            motor3.throttle = 0
                            diff_pitch = get_pitch() - pitch
                            pos2[2] += 110 * abs(np.tan(diff_pitch * np.pi / 180))
                            print("motor2 is on (CCW)")
                            print("pos2:", pos2[2])

                    if pitch == 0.0:
                        print("pitch adjusted")
                        print("pos1:", pos1[2], " pos2:", pos2[2], " pos3:", pos3[2])
                        pitch_end = time.time() - pitch_time
                        print("adjusted in:", pitch_end, " seconds")
                        motor1.throttle = 0
                        motor2.throttle = 0
                        motor3.throttle = 0
                        balance_pitch = True

    except KeyboardInterrupt:
        print("Exiting")


main()
motor1.throttle = 0
motor2.throttle = 0
motor3.throttle = 0
