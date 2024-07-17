#!/usr/bin/env python3
# Constant frequency for prediction update
# Getting linear velocity and angular velocity from /odom topic for better results
# Yaw error is not measures as we dont have a compass onboard

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from ekf_localizer.msg import ErrorWithBounds
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class LowPassFilter:
    def __init__(self, alpha):
        self.alpha = alpha
        self.filtered_value = None

    def filter(self, value):
        if self.filtered_value is None:
            self.filtered_value = value
        else:
            self.filtered_value = self.alpha * value + (1 - self.alpha) * self.filtered_value
        return self.filtered_value

class EKF:
    def __init__(self):
        self.dt = 0.1  # Time step, can be adjusted based on your IMU message frequency
        
        # State vector [x, y, theta]
        self.x = np.zeros(3)
        
        # Covariance matrix
        self.P = np.eye(3)
        
        # Process noise covariance
        self.Q = np.diag([0.5, 0.5, 0.5])  # Increase these values to decrease confidence in IMU measurements
        
        # Measurement noise covariance for GPS
        self.R_gps = np.diag([1.0, 1.0])

        # Control noise covariance
        self.sigma_v = 0
        self.sigma_w = 0
        self.qu = np.diag([self.sigma_v, self.sigma_w])

    def predict(self, vel, omega):
        theta = self.x[2]
        
        for i in range(25):  # Iterating to integrate over smaller time steps
            # State transition model
            F = np.array([
                [1, 0, -vel * np.sin(theta) * self.dt/25],
                [0, 1,  vel * np.cos(theta) * self.dt/25],
                [0, 0, 1]
            ])
            
            # Control input model
            B = np.array([
                [np.cos(theta) * self.dt/25, 0],
                [np.sin(theta) * self.dt/25, 0],
                [0, self.dt/25]
            ])
            
            u = np.array([vel, omega])
            
            # Predict the state
            self.x = self.x + np.dot(B, u)
            self.x[2] = (self.x[2] + np.pi) % (2 * np.pi) - np.pi  # Normalize angle
            
            # Predict the covariance
            G = np.array([
                [np.cos(self.x[2]), 0],
                [np.sin(self.x[2]), 0],
                [0, 1]
            ])
            Q = G @ self.qu @ G.T
            self.P = np.dot(F, np.dot(self.P, F.T)) + Q

    def update_gps(self, z):
        # Measurement model
        H = np.array([
            [1, 0, 0],
            [0, 1, 0]
        ])
        
        # Measurement prediction
        z_pred = np.dot(H, self.x)
        
        # Measurement residual
        y = z - z_pred
        
        # Measurement covariance
        S = np.dot(H, np.dot(self.P, H.T)) + self.R_gps
        
        # Kalman gain
        K = np.dot(self.P, np.dot(H.T, np.linalg.inv(S)))
        
        # Update the state
        self.x = self.x + np.dot(K, y)
        
        # Update the covariance
        I = np.eye(len(self.P))
        self.P = np.dot((I - np.dot(K, H)), self.P)

class EKFNode:
    def __init__(self):
        rospy.init_node('ekf_node')
        
        self.ekf = EKF()
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.gps_sub = rospy.Subscriber('/local_coordinates', PoseStamped, self.gps_callback)
        self.pose_sub = rospy.Subscriber('/pose', PoseStamped, self.pose_callback)
        self.pose_pub = rospy.Publisher('/ekf_pose', PoseStamped, queue_size=10)
        # self.local_pub = rospy.Publisher('/local_coordinates_ekf', PoseStamped, queue_size=10)
        self.error_pub = rospy.Publisher('/ekf_error', ErrorWithBounds, queue_size=10)
        # self.local_pose_pub = rospy.Publisher('/local_pose', PoseStamped, queue_size=10)
        
        self.vel_filter = LowPassFilter(alpha=0.1)
        self.omega_filter = LowPassFilter(alpha=0.1)

        # Initial pose
        self.initial_pose = None

        # Timer for prediction
        self.dt = 0.1  # 10 Hz
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.timer_callback)
        
        rospy.spin()

    def odom_callback(self, msg):
        # Extract and filter velocity and angular velocity
        raw_vel = msg.twist.twist.linear.x  # Linear velocity in x direction
        raw_omega = msg.twist.twist.angular.z  # Angular velocity around z-axis
        rospy.loginfo(f"Raw AngularVelocity={raw_omega}")
        
        self.vel = self.vel_filter.filter(raw_vel)
        self.omega = self.omega_filter.filter(raw_omega)
        
        # Optional: log the filtered values
        rospy.loginfo(f"Filtered LinearVelocity={self.vel}")
        rospy.loginfo(f"Filtered AngularVelocity={self.omega}")

    def timer_callback(self, event):
        # EKF prediction step
        self.ekf.predict(self.vel, self.omega)
        
        # Publish the predicted state
        self.publish_pose()

    def gps_callback(self, msg):
        # Extract GPS position
        z = np.array([msg.pose.position.x, msg.pose.position.y])
        
        # EKF update step
        self.ekf.update_gps(z)
        
        # Publish the updated state
        self.publish_pose()
        
        # Calculate and publish error and 3-sigma values
        self.publish_error(z)

    def pose_callback(self, msg):
        if self.initial_pose is None:
            self.initial_pose = msg.pose
        else:
            # Calculate the local pose relative to the initial pose
            local_pose = PoseStamped()
            local_pose.header.stamp = rospy.Time.now()
            local_pose.header.frame_id = "map"
            
            # Transform current pose to the local frame
            local_pose.pose.position.x = msg.pose.position.x - self.initial_pose.position.x
            local_pose.pose.position.y = msg.pose.position.y - self.initial_pose.position.y
            local_pose.pose.position.z = msg.pose.position.z - self.initial_pose.position.z

            # Handle orientation: Convert quaternion to Euler angles for calculation
            initial_orientation = self.initial_pose.orientation
            initial_euler = euler_from_quaternion([
                initial_orientation.x, initial_orientation.y,
                initial_orientation.z, initial_orientation.w
            ])
            
            current_orientation = msg.pose.orientation
            current_euler = euler_from_quaternion([
                current_orientation.x, current_orientation.y,
                current_orientation.z, current_orientation.w
            ])

            # Calculate relative yaw
            relative_yaw = current_euler[2] - initial_euler[2]
            local_quat = quaternion_from_euler(0, 0, relative_yaw)
            
            local_pose.pose.orientation.x = local_quat[0]
            local_pose.pose.orientation.y = local_quat[1]
            local_pose.pose.orientation.z = local_quat[2]
            local_pose.pose.orientation.w = local_quat[3]
            
            self.local_pose_pub.publish(local_pose)

    def publish_pose(self):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = self.ekf.x[0]
        pose_msg.pose.position.y = self.ekf.x[1]
        pose_msg.pose.position.z = 0
        pose_msg.pose.orientation.w = np.cos(self.ekf.x[2] / 2.0)
        pose_msg.pose.orientation.z = np.sin(self.ekf.x[2] / 2.0)
        self.pose_pub.publish(pose_msg)

        local_pose_msg = PoseStamped()
        local_pose_msg.header.stamp = rospy.Time.now()
        local_pose_msg.header.frame_id = "local_coordinates"
        local_pose_msg.pose.position.x = self.ekf.x[0]
        local_pose_msg.pose.position.y = self.ekf.x[1]
        local_pose_msg.pose.position.z = 0
        local_pose_msg.pose.orientation.w = np.cos(self.ekf.x[2] / 2.0)
        local_pose_msg.pose.orientation.z = np.sin(self.ekf.x[2] / 2.0)
        self.local_pub.publish(local_pose_msg)

    def publish_error(self, z):
        error_x = z[0] - self.ekf.x[0]
        error_y = z[1] - self.ekf.x[1]
        error_yaw = 0  # Placeholder, as GPS does not provide yaw
        
        sigma_x = 3 * np.sqrt(self.ekf.P[0, 0])
        sigma_y = 3 * np.sqrt(self.ekf.P[1, 1])
        sigma_yaw = 3 * np.sqrt(self.ekf.P[2, 2])
        
        error_msg = ErrorWithBounds()
        error_msg.error_x = error_x
        error_msg.error_y = error_y
        error_msg.error_yaw = error_yaw
        error_msg.sigma_x_pos = sigma_x
        error_msg.sigma_x_neg = -sigma_x
        error_msg.sigma_y_pos = sigma_y
        error_msg.sigma_y_neg = -sigma_y
        error_msg.sigma_yaw_pos = sigma_yaw
        error_msg.sigma_yaw_neg = -sigma_yaw
        self.error_pub.publish(error_msg)

if __name__ == '__main__':
    try:
        EKFNode()
    except rospy.ROSInterruptException:
        pass
