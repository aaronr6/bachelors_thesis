import matplotlib.pyplot as plt
import numpy as np

# Load data from file
data = np.loadtxt('end_effector_states.txt')

# Extract time, positions, orientations, and reference values
time = data[:, 0]
x = data[:, 1]
y = data[:, 2]
z = data[:, 3]
roll = data[:, 4]
pitch = data[:, 5]
yaw = data[:, 6]
x_ref = data[:, 7]
y_ref = data[:, 8]
z_ref = data[:, 9]
roll_ref = data[:, 10]
pitch_ref = data[:, 11]
yaw_ref = data[:, 12]

# Labels and y-axis labels
labels = ['X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw']
ylabels = ['Position [m]', 'Orientation [rad]']

# Plot the TCP position over time
plt.figure()
plt.plot(time, x, label='X')
plt.plot(time, x_ref, label='X_ref', linestyle='--')
plt.xlabel('Time [s]')
plt.ylabel(ylabels[0])
plt.legend()
plt.title('TCP X Position Over Time')
plt.grid()
plt.show()

plt.figure()
plt.plot(time, y, label='Y')
plt.plot(time, y_ref, label='Y_ref', linestyle='--')
plt.xlabel('Time [s]')
plt.ylabel(ylabels[0])
plt.legend()
plt.title('TCP Y Position Over Time')
plt.grid()
plt.show()

plt.figure()
plt.plot(time, z, label='Z')
plt.plot(time, z_ref, label='Z_ref', linestyle='--')
plt.xlabel('Time [s]')
plt.ylabel(ylabels[0])
plt.legend()
plt.title('TCP Z Position Over Time')
plt.grid()
plt.show()

# Plot the TCP orientation over time
plt.figure()
plt.plot(time, roll, label='Roll')
plt.plot(time, roll_ref, label='Roll_ref', linestyle='--')
plt.xlabel('Time [s]')
plt.ylabel(ylabels[1])
plt.legend()
plt.title('TCP Roll Over Time')
plt.grid()
plt.show()

plt.figure()
plt.plot(time, pitch, label='Pitch')
plt.plot(time, pitch_ref, label='Pitch_ref', linestyle='--')
plt.xlabel('Time [s]')
plt.ylabel(ylabels[1])
plt.legend()
plt.title('TCP Pitch Over Time')
plt.grid()
plt.show()

plt.figure()
plt.plot(time, yaw, label='Yaw')
plt.plot(time, yaw_ref, label='Yaw_ref', linestyle='--')
plt.xlabel('Time [s]')
plt.ylabel(ylabels[1])
plt.legend()
plt.title('TCP Yaw Over Time')
plt.grid()
plt.show()