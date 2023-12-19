import pandas as pd
import math
import numpy as np
import matplotlib.pyplot as plt
from utilities import FileReader

# Plotting function (taken from rrt.py)
def plot_circle(x, y, size, color="black"):  # pragma: no cover
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, color)

# Read in data
path_data = pd.read_csv('pathData.csv')
robot_pose = pd.read_csv('robotPose.csv')
linear_data = pd.read_csv('linear.csv')
angular_data = pd.read_csv('angular.csv')

# Clean up data, for some reason there are spaces in the column names
robot_pose.columns = robot_pose.columns.str.strip()
linear_data.columns = linear_data.columns.str.strip()
angular_data.columns = angular_data.columns.str.strip()

# Change epoch time to delta time in seconds
robot_pose['delta_time'] = (robot_pose['stamp'] - robot_pose['stamp'].iloc[0]) / 1e9
linear_data['delta_time'] = (linear_data['stamp'] - linear_data['stamp'].iloc[0]) / 1e9
angular_data['delta_time'] = (angular_data['stamp'] - angular_data['stamp'].iloc[0]) / 1e9

# Obstacles
# obstacle_list = [
#     (1, 2.5, 1),
#     (6, 2.5, 1),
#     (11, 2.5, 1),
#     (3.5, 5, 1),
#     (8.5, 5, 1),
#     (1, 7.5, 1),
#     (6, 7.5, 1),
#     (11, 7.5, 1),
# ]  # [x,y,size(radius)]

obstacle_list = [
    (5, 5, 1),
    (3, 6, 2),
    (3, 8, 2),
    (3, 10, 2),
    (7, 5, 2),
    (9, 5, 2),
    (8, 10, 1),
    (6, 12, 1),
]  # [x,y,size(radius)]

# Plot 1: Path with obstacles and labels!
plt.figure(figsize=(10, 6))
plt.plot(path_data['path_x'].values, path_data['path_y'].values, label='Desired Path', color='green')
plt.plot(robot_pose['kf_x'].values, robot_pose['kf_y'].values, label='Robot Path', color='blue')
start_x, start_y = path_data['path_x'].iloc[0], path_data['path_y'].iloc[0]
end_x, end_y = path_data['path_x'].iloc[-1], path_data['path_y'].iloc[-1]
plt.scatter([start_x, end_x], [start_y, end_y], color='red')

plt.text(start_x, start_y, 'Start', fontsize=12, verticalalignment='bottom')
plt.text(end_x, end_y, 'End', fontsize=12, verticalalignment='bottom')

for (ox, oy, size) in obstacle_list:
    plot_circle(ox, oy, size)

plt.title('Path')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.legend()
plt.grid(True)
plt.show()

# Robot pose time goes on longer than linear and angular data, so we need to trim it
max_linear_delta_time = linear_data['delta_time'].max()
robot_pose = robot_pose[robot_pose['delta_time'] <= max_linear_delta_time]

# Plot 2: Kalman Filter Data
robot_pose_t = robot_pose['delta_time'].values
plt.figure(figsize=(10, 6))
plt.plot(robot_pose_t, robot_pose['imu_ax'].values, label='imu_ax')
plt.plot(robot_pose_t, robot_pose['imu_ay'].values, label='imu_ay')
plt.plot(robot_pose_t, robot_pose['kf_ax'].values, label='kf_ax')
plt.plot(robot_pose_t, robot_pose['kf_ay'].values, label='kf_ay')
plt.plot(robot_pose_t, robot_pose['kf_vx'].values, label='kf_vx')
plt.plot(robot_pose_t, robot_pose['kf_w'].values, label='kf_w')
plt.title('Kalman Filter Data Over Time')
plt.xlabel('Time (s)')
plt.ylabel('Values')
plt.legend()
plt.grid(True)
plt.show()

# Plot 3: Linear and Angular Values
fig, axs = plt.subplots(2, 1, figsize=(10, 12))

# Linear Plot
linear_data_t = linear_data['delta_time'].values
axs[0].plot(linear_data_t, linear_data['e'].values, label='e')
axs[0].plot(linear_data_t, linear_data['e_dot'].values, label='e_dot')
axs[0].plot(linear_data_t, linear_data['e_int'].values, label='e_int')
axs[0].set_title('Linear Values Over Time')
axs[0].set_xlabel('Time (s)')
axs[0].set_ylabel('Values')
axs[0].legend()
axs[0].grid(True)

# Angular Plot
angular_data_t = angular_data['delta_time'].values
axs[1].plot(angular_data_t, angular_data['e'].values, label='e')
axs[1].plot(angular_data_t, angular_data['e_dot'].values, label='e_dot')
axs[1].plot(angular_data_t, angular_data['e_int'].values, label='e_int')
axs[1].set_title('Angular Values Over Time')
axs[1].set_xlabel('Time (s)')
axs[1].set_ylabel('Values')
axs[1].legend()
axs[1].grid(True)

plt.tight_layout()
plt.show()
