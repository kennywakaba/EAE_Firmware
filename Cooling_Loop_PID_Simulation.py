import numpy as np
import matplotlib.pyplot as plt

# Simulation parameters
dt = 0.1                # time step (s)
time = np.arange(0, 10, dt)  # simulate for 10 seconds

# PID parameters
Kp = 1.2
Ki = 0.4
Kd = 0.05

# System parameters (first-order system)
tau = 2.0  # time constant
setpoint = 0

# Initialize variables
output = 0
integral = 0
prev_error = 0
measured = 5  # initial temp

# Data storage
measured_vals = []

# Simulation loop
for t in time:
    error = setpoint - measured
    integral += error * dt
    derivative = (error - prev_error) / dt
    control = Kp * error + Ki * integral + Kd * derivative
    prev_error = error

    # Simulate plant response (1st order lag)
    measured += (control - measured) * dt / tau
    measured_vals.append(measured)

# Plotting
plt.figure(figsize=(10, 5))
plt.plot(time, measured_vals, label="Measured Temperature")
plt.axhline(setpoint, color='r', linestyle='--', label="Setpoint")
plt.title("PID Control Step Response")
plt.xlabel("Time (s)")
plt.ylabel("Temperature")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()
