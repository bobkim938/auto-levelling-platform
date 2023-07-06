kp = 5
ki = 0.01
kd = 0.01
pasterror = 0.0
error = 0.0
max_pid = 0.5
min_pid = -0.5
tau = 3
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


a = []
for i in range(5, 0, -1):
    a.append(pid(i, 0.1))

print(a)
