import busio
import board
from adafruit_pca9685 import PCA9685
import adafruit_motor.servo
from mpu6050 import mpu6050
import numpy as np
import time
from Kalman_Filter import KalmanFilter
from Calibration_IMU import CalibrationIMU


kfr = KalmanFilter()  # initialize Kalman Filter
kfp = KalmanFilter()
sensor = mpu6050(0x68)
i2c = busio.I2C(board.SCL, board.SDA)  # Set up the I2C bus using GPIO 2, 3
pca = PCA9685(i2c)
pca.frequency = 50  # Set the PWM frequency to 50hz
motor1 = adafruit_motor.servo.ContinuousServo(pca.channels[0])  # Set the servo to channel 0
motor2 = adafruit_motor.servo.ContinuousServo(pca.channels[2])  # Set the servo to channel 1
motor3 = adafruit_motor.servo.ContinuousServo(pca.channels[4])  # Set the servo to channel 2
kp = 0.1
ki = 0.01
kd = 0.001
pasterror = 0.0
error = 0.0
max_pid = 1
min_pid = -1
tau = 0.0
sample_t = 0.0
proportional = 0.0
integral = 0.0
derivative = 0.0
prevmeasurement = 0.0
pid_output = 0.0
max_rp = 5
min_rp = -5


def pid(measurement, dt, setpoint=0):
    global kp, ki, kd
    global error, pasterror
    global max_pid, min_pid  # for integrator anti-windup
    global tau, sample_t
    global proportional, integral, derivative, prevmeasurement
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
    tau = sample_t * 0.5
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

    derivative = ((2 * kd) / (sample_t + 2 * tau)) * (measurement - prevmeasurement) - (
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
    prevmeasurement = measurement
    result = abs(pid_output)/78

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
    pitch = 0
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
        p = kfp.get_angle(pitch, gyro_y, dt) - 2.5
        pitch += p
    pitch = pitch / 5
    return pitch


def main():
    roll = 0.0
    pitch = 0.0
    # boolean variables for platform balance
    balance_roll = True
    balance_pitch = True
    a = False
    b = False
    # position of the servos
    pos1 = np.array([0, 127, 127 * np.sin(roll * np.pi / 180)])
    pos2 = np.array([110, -63.5, 110 * np.sin(pitch * np.pi / 180)])
    pos3 = np.array([-110, -63.5, 110 * np.sin(pitch * np.pi / 180)])
    motor_constant = 1
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
                temp_x = roll
                while not balance_roll:
                    print("adjusting roll")
                    start_time = time.time()
                    roll = int(get_roll())
                    if roll > 0.0:
                        print("roll is positive")
                        if pos1[2] == 0 and pos2[2] == 0 and pos3[2] == 0:
                            a = True
                            motor_constant = 1
                            dt = time.time() - start_time
                            motor1.throttle = motor_constant * pid(roll, dt)
                            motor2.throttle = 0
                            motor3.throttle = 0
                            print("motor1 is on (CCW)")
                        elif pos2[2] >= (127 * np.sin(roll * np.pi / 180)) and pos3[2] >= (
                                127 * np.sin(roll * np.pi / 180)):
                            b = True
                            motor_constant *= -1
                            dt = time.time() - start_time
                            motor1.throttle = 0
                            motor2.throttle = motor_constant * pid(roll, dt)
                            motor3.throttle = motor_constant * pid(roll, dt)
                            print("motor2 and motor3 are on (CW)")
                        else:
                            a = True
                            motor_constant = 1
                            dt = time.time() - start_time
                            motor1.throttle = motor_constant * pid(roll, dt)
                            motor2.throttle = 0
                            motor3.throttle = 0
                            print("motor1 is on (CCW)")
                    elif roll < 0.0:
                        print("roll is negative")
                        temp_x = roll
                        if pos2[2] == 0 and pos3[2] == 0 and pos1[2] == 0:
                            a = True
                            motor_constant = 1
                            dt = time.time() - start_time
                            pid_r = pid(roll, dt)
                            motor1.throttle = 0
                            motor2.throttle = motor_constant * pid_r
                            motor3.throttle = motor_constant * pid_r
                            print("motor2 and motor3 are on (CCW)")
                        elif pos1[2] >= (127 * np.sin(roll * np.pi / 180)):
                            b = True
                            motor_constant *= -1
                            dt = time.time() - start_time
                            motor1.throttle = motor_constant * pid(roll, dt)
                            motor2.throttle = 0
                            motor3.throttle = 0
                            print("motor1 is on (CW)")
                        else:
                            a = True
                            motor_constant = 1
                            dt = time.time() - start_time
                            pid_r = pid(roll, dt)
                            motor1.throttle = 0
                            motor2.throttle = motor_constant * pid_r
                            motor3.throttle = motor_constant * pid_r
                            print("motor2 and motor3 are on (CCW)")

                    if roll == 0.0:
                        print("roll adjusted")
                        motor1.throttle = 0
                        motor2.throttle = 0
                        motor3.throttle = 0
                        motor_constant = 1
                        balance_roll = True
                        if temp_x > 0.0:
                            if a:
                                pos1[2] += 127 * np.sin(temp_x * np.pi / 180)
                                a = False
                                print("pos1 adjusted")
                            elif b:
                                pos2[2] -= 127 * np.sin(temp_x * np.pi / 180)
                                pos3[2] -= 127 * np.sin(temp_x * np.pi / 180)
                                b = False
                                print("pos2 and pos3 adjusted")
                        elif temp_x < 0.0:
                            if a:
                                pos2[2] += -1 * 127 * np.sin(temp_x * np.pi / 180)
                                pos3[2] += -1 * 127 * np.sin(temp_x * np.pi / 180)
                                a = False
                                print("pos2 and pos3 adjusted")
                            elif b:
                                pos1 += 127 * np.sin(temp_x * np.pi / 180)
                                b = False
                                print("pos1 adjusted")

            if not balance_pitch:
                start_time = time.time()
                temp_y = pitch
                while not balance_pitch:
                    print("adjusting pitch")
                    pitch = int(get_pitch())
                    if pitch > 0.0:
                        print("pitch is positive")
                        if pos3[2] == 0 and pos2[2] == 0:
                            a = True
                            dt = time.time() - start_time
                            motor1.throttle = 0
                            motor2.throttle = 0
                            motor3.throttle = motor_constant * pid(pitch, dt)
                            print("motor3 is on (CCW)")
                        elif pos2[2] >= (110 * np.sin(pitch * np.pi / 180)):
                            b = True
                            motor_constant *= -1
                            dt = time.time() - start_time
                            motor1.throttle = 0
                            motor2.throttle = motor_constant * pid(pitch, dt)
                            motor3.throttle = 0
                            print("motor2 is on (CW)")
                        else:
                            a = True
                            dt = time.time() - start_time
                            motor1.throttle = 0
                            motor2.throttle = 0
                            motor3.throttle = motor_constant * pid(pitch, dt)
                            print("motor3 is on (CCW)")
                    elif pitch < 0.0:
                        print("pitch is negative")
                        temp_y = pitch
                        if pos2[2] == 0 and pos3[2] == 0:
                            a = True
                            dt = time.time() - start_time
                            motor1.throttle = 0
                            motor2.throttle = motor_constant * pid(pitch, dt)
                            motor3.throttle = 0
                            print("motor2 is on (CCW)")
                        elif pos3[2] >= (110 * np.sin(pitch * np.pi / 180)):
                            b = True
                            motor_constant *= -1
                            dt = time.time() - start_time
                            motor1.throttle = 0
                            motor2.throttle = 0
                            motor3.throttle = motor_constant * pid(pitch, dt)
                            print("motor3 is on (CW)")
                        else:
                            a = True
                            dt = time.time() - start_time
                            motor1.throttle = 0
                            motor2.throttle = motor_constant * pid(pitch, dt)
                            motor3.throttle = 0
                            print("motor2 is on (CCW)")

                    if pitch == 0.0:
                        print("pitch adjusted")
                        motor1.throttle = 0
                        motor2.throttle = 0
                        motor3.throttle = 0
                        motor_constant = 1
                        balance_pitch = True
                        if temp_y > 0.0:
                            if a:
                                pos3[2] += 110 * np.sin(temp_y * np.pi / 180)
                                a = False
                                print("pos3 adjusted")
                            elif b:
                                pos2[2] -= 110 * np.sin(temp_y * np.pi / 180)
                                b = False
                                print("pos2 adjusted")
                        elif temp_y < 0.0:
                            if a:
                                pos2[2] += -1 * 110 * np.sin(temp_y * np.pi / 180)
                                a = False
                                print("pos2 adjusted")
                            elif b:
                                pos3[2] += 110 * np.sin(temp_y * np.pi / 180)
                                b = False
                                print("pos3 adjusted")

    except KeyboardInterrupt:
        print("Exiting")


main()
motor1.throttle = 0
motor2.throttle = 0
motor3.throttle = 0