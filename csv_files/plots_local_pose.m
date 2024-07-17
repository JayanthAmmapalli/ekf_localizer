% Clear workspace and command window
clc;
clear all;

% Import CSV files as tables
pose = readtable("test_output_ekf_v3_LP_4-pose.csv");
local_pose = readtable("test_output_ekf_v3_LP_4-local_pose.csv");
local_gps = readtable("test_output_ekf_v3_LP_4-local_coordinates.csv");

% Plot data from the 'pose' table
figure;
hold on
plot(pose.x_pose_position_x, pose.x_pose_position_y, 'LineWidth', 2);
plot(local_gps.x_pose_position_x, local_gps.x_pose_position_y, 'LineWidth', 2);
% plot(pose.x_pose_position_x, pose.x_pose_position_y, 'LineWidth', 2);
title('Pose Position');
xlabel('Position X');
ylabel('Position Y');
grid on;
legend('Pose Trajectory');

% Optionally, you can plot other tables in the same figure or separate figures
% Example for plotting 'local_pose' and 'local_gps'
% figure;
% hold on;
% plot(local_pose.x_local_pose_position_x, local_pose.x_local_pose_position_y, 'LineWidth', 2);
% plot(local_gps.x_local_coordinates_x, local_gps.x_local_coordinates_y, 'LineWidth', 2);
% title('Local Pose and GPS Coordinates');
% xlabel('Position X');
% ylabel('Position Y');
% grid on;
% legend('Local Pose', 'Local GPS');
% hold off;
