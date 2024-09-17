import matplotlib.pyplot as plt
import numpy as np

# Load data from file
data = np.loadtxt('../build/tcp_positions.txt')

# Extract time and positions
time = data[:, 0]
# x = data[:, 1]
# y = data[:, 2]
# z = data[:, 3]
# x_d = data[:, 4]
# y_d = data[:, 5]
# z_d = data[:, 6]
# roll = data[:, 7]
# pitch = data[:, 8]
# yaw = data[:, 9]
# roll_d = data[:, 10]
# pitch_d = data[:, 11]
# yaw_d = data[:, 12]

# Labels and y-axis labels
labels = ['X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw']
desired_labels = ['X_ref', 'Y_ref', 'Z_ref', 'Roll_ref', 'Pitch_ref', 'Yaw_ref']
current_labels = ['X_d', 'Y_d', 'Z_d', 'Roll_d', 'Pitch_d', 'Yaw_d']
ylabels = ['Position [m]', 'Orientation [rad]']

# Plot the TCP coordinates over time
for i in range(3):
    plt.figure()
    plt.plot(time, data[:, i + 1], label=labels[i])
    plt.plot(time, data[:, i + 4], label=desired_labels[i], linestyle='--')
    plt.plot(time, data[:, i + 13], label=current_labels[i], linestyle='dashdot')
    plt.xlabel('Time [s]')
    plt.ylabel(ylabels[0])
    plt.title(f'TCP {labels[i]} Position Over Time')
    plt.legend()
    plt.grid()
    plt.show()

# Plot the TCP orientation over time
for i in range(3, 6):
    plt.figure()
    plt.plot(time, data[:, i + 4], label=labels[i])
    plt.plot(time, data[:, i + 7], label=desired_labels[i], linestyle='--')
    plt.plot(time, data[:, i + 13], label=current_labels[i], linestyle='dashdot')
    plt.xlabel('Time [s]')
    plt.ylabel(ylabels[1])
    plt.title(f'TCP {labels[i]} Orientation Over Time')
    plt.legend()
    plt.grid()
    plt.show()