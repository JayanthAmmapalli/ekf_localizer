% Clear workspace and command window
clc;
clear all;

% Import CSV files as tables
pose = readtable("test_output_ekf_v3_LP_5-pose.csv");
local_pose = readtable("test_output_ekf_v3_LP_5-local_pose.csv");
local_gps = readtable("test_output_ekf_v3_LP_5-local_coordinates.csv");
ekf_pose = readtable("test_output_ekf_v3_LP_5-ekf_pose.csv");
ekf_error  = readtable('test_output_ekf_v3_LP_5-ekf_error.csv');



intial_time = local_gps.x_header_stamp_secs(1) + local_gps.x_header_stamp_nsecs(1)/10^(9);
time = local_gps.x_header_stamp_secs + local_gps.x_header_stamp_nsecs/10^(9);
time = time - intial_time;



% Plot data from the 'pose' table
figure(1);
hold on
plot(-local_pose.x_pose_position_y, local_pose.x_pose_position_x, 'LineWidth', 1,'Color','g');
plot(local_gps.x_pose_position_x, local_gps.x_pose_position_y, 'LineWidth', 1,'Color','r');
plot(ekf_pose.x_pose_position_x,ekf_pose.x_pose_position_y,'LineWidth', 1,'Color','b');
% plot(pose.x_pose_position_x, pose.x_pose_position_y, 'LineWidth', 2);
title('Odom Frame');
xlabel('X(m)');
ylabel('Y(m)');
grid on;
hold off;
legend('local-rosbot-ekf','utm-local','efk-local');

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
figure(2);
subplot(3,1,1)
hold on;
plot(time,ekf_error.x_error_x,'LineWidth', 1,'Color','r');
plot(time,ekf_error.x_sigma_x_pos,'LineWidth', 1,'Color','b');
plot(time,ekf_error.x_sigma_x_neg,'LineWidth', 1,'Color','b');
ylabel('X(m)');
hold off;
grid on
subplot(3,1,2)
hold on;
plot(time,ekf_error.x_error_y,'LineWidth', 1,'Color','r');
plot(time,ekf_error.x_sigma_y_pos,'LineWidth', 1,'Color','b');
plot(time,ekf_error.x_sigma_y_neg,'LineWidth', 1,'Color','b');
ylabel('Y(m)');
hold off;
grid on
subplot(3,1,3)
hold on;
plot(time,ekf_error.x_error_yaw,'LineWidth', 1,'Color','r');
plot(time,ekf_error.x_sigma_yaw_pos,'LineWidth', 1,'Color','b');
plot(time,ekf_error.x_sigma_yaw_neg,'LineWidth', 1,'Color','b');
ylabel('Yaw');
hold off;
grid on

