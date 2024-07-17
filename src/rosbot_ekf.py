#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import PoseStamped
import utm

class EKFLocalizer:
    def __init__(self):
        rospy.init_node('ekf_localizer')
        
        self.gps_sub = rospy.Subscriber('/ublox/fix', NavSatFix, self.gps_callback)
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)
        self.pose_pub = rospy.Publisher('/EKF_pose', PoseStamped, queue_size=10)
        
        self.first_gps_received = False
        self.origin_utm = None
        
        # EKF state and covariance
        self.x_hat = np.zeros(3)  # [x, y, yaw]
        self.P = np.eye(3) * 0.1
        
        # Process noise covariance
        self.Q = np.diag([0.1, 0.1, 0.05])
        
        # Measurement noise covariance
        self.R_gps = np.diag([1.0, 1.0])
        
        # Previous time
        self.prev_time = rospy.Time.now()
        
    def gps_callback(self, msg):
        if not self.first_gps_received:
            self.origin_utm = utm.from_latlon(msg.latitude, msg.longitude)[:2]
            self.first_gps_received = True
            self.x_hat[0], self.x_hat[1] = self.origin_utm
        else:
            current_utm = utm.from_latlon(msg.latitude, msg.longitude)[:2]
            self.gps_update(current_utm)

    def imu_callback(self, msg):
        current_time = rospy.Time.now()
        dt = (current_time - self.prev_time).to_sec()
        self.prev_time = current_time
        
        angular_velocity = msg.angular_velocity.z
        rospy.loginfo(f"Angular_velocity = {angular_velocity}")
        linear_velocity = np.sqrt(msg.linear_acceleration.x**2 + msg.linear_acceleration.y**2)
        
        self.prediction_step(linear_velocity, angular_velocity, dt)
        self.publish_pose()
        
    def prediction_step(self, v, omega, dt):
        x, y, yaw = self.x_hat
        
        # State prediction
        self.x_hat[0] += v * np.cos(yaw) * dt
        self.x_hat[1] += v * np.sin(yaw) * dt
        self.x_hat[2] += omega * dt
        
        # Jacobian of the motion model
        F = np.array([
            [1, 0, -v * np.sin(yaw) * dt],
            [0, 1,  v * np.cos(yaw) * dt],
            [0, 0, 1]
        ])
        
        # Process noise
        G = np.array([
            [np.cos(yaw) * dt, 0],
            [np.sin(yaw) * dt, 0],
            [0, dt]
        ])
        
        # Covariance prediction
        self.P = F @ self.P @ F.T + G @ self.Q @ G.T
    
    def gps_update(self, current_utm):
        x, y = current_utm
        z = np.array([x, y])
        
        # Measurement model
        H = np.array([
            [1, 0, 0],
            [0, 1, 0]
        ])
        
        # Measurement prediction
        z_hat = H @ self.x_hat
        
        # Innovation covariance
        S = H @ self.P @ H.T + self.R_gps
        
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # State update
        self.x_hat = self.x_hat + K @ (z - z_hat)
        
        # Covariance update
        self.P = (np.eye(3) - K @ H) @ self.P
    
    def publish_pose(self):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = self.x_hat[0]
        pose_msg.pose.position.y = self.x_hat[1]
        pose_msg.pose.position.z = 0
        pose_msg.pose.orientation.z = np.sin(self.x_hat[2] / 2)
        pose_msg.pose.orientation.w = np.cos(self.x_hat[2] / 2)
        
        self.pose_pub.publish(pose_msg)

if __name__ == '__main__':
    try:
        ekf = EKFLocalizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
