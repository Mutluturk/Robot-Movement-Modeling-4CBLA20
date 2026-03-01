import numpy as np
import matplotlib.pyplot as plt


m = 2             # estimated robot mass [kg] (adjust if needed)
r = 0.055        # wheel radius [m]
g = 9.81
mu = 0.6

tau = 1.08           # Nm (servo torque)
rpm_servo = 71.4     # rpm (from calculation)

# Convert rpm to rad/s
omega_max = rpm_servo * 2*np.pi / 60
v_max = omega_max * r

# Ramp angle
alpha = np.deg2rad(20)

# Simulation
dt = 0.001
T = 5
t = np.arange(0, T, dt)

x = 0
v = 0

x_list = []
v_list = []

for _ in t:

    # Forces
    F_motor = 2 * tau / r
    F_gravity = m * g * np.sin(alpha)
    N = m * g * np.cos(alpha)
    F_max = mu * N

    # Friction limit
    F_drive = min(F_motor, F_max)

    F_net = F_drive - F_gravity
    a = F_net / m

    # Integrate
    v += a * dt

    # Speed limit from servo rpm
    if v > v_max:
        v = v_max

    x += v * dt

    x_list.append(x)
    v_list.append(v)

# Plot
plt.figure()
plt.plot(t, x_list)
plt.xlabel("Time (s)")
plt.ylabel("Position (m)")
plt.title("1:1 Gear Ratio - Ramp Climb")
plt.grid()

plt.figure()
plt.plot(t, v_list)
plt.xlabel("Time (s)")
plt.ylabel("Velocity (m/s)")
plt.title("Velocity (limited by servo speed)")
plt.grid()

plt.show()