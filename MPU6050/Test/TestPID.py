import numpy as np
import matplotlib.pyplot as plt

# Define the transfer function
num = [1]
den = [1, 3, 2]
G = np.poly1d(num)/np.poly1d(den)

# Define the PID controller
Kp = 0.6
Ki = 0.03
Kd = 0.5
s = np.poly1d([1, 0])
C = Kd*s**2 + Kp*s + Ki

# Define the initial condition
x0 = -15

# Define the setpoint
SP = 0

# Define the time vector
t = np.linspace(0, 10, 1000)

# Simulate the system
y = np.zeros_like(t)
x = x0
for i in range(len(t)):
    e = SP - x
    u = C(e)
    xdot = G(x) + u
    x = x + xdot*(t[i]-t[i-1])
    y[i] = x

# Modify the y-axis parameters
y = y + SP

# Plot the step response
plt.plot(t, y)
plt.xlabel('Time (s)')
plt.ylabel('Degrees')
plt.title('Step Response with PID Control')
plt.grid()
plt.show()