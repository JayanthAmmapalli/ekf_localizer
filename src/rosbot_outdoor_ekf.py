#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from ekf_localizer.msg import UTMCoordinates
import utm

class GpsToUtmConverter:
    def __init__(self):
        rospy.init_node('gps_to_utm_node', anonymous=True)
        
        self.pub_utm = rospy.Publisher('utm_coordinates', UTMCoordinates, queue_size=10)
        self.pub_pose = rospy.Publisher('local_coordinates', PoseStamped, queue_size=10)
        
        rospy.Subscriber('/ublox/fix', NavSatFix, self.gps_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        self.home_utm = None
        self.home_pose = None
        
        rospy.spin()

    def gps_callback(self, data):
        utm_coords = utm.from_latlon(data.latitude, data.longitude)
        if self.home_utm is None:
            self.home_utm = utm_coords

        utm_msg = UTMCoordinates()
        utm_msg.x = utm_coords[0] - self.home_utm[0]
        utm_msg.y = utm_coords[1] - self.home_utm[1]
        utm_msg.zone = f"{utm_coords[2]}{utm_coords[3]}"
        self.pub_utm.publish(utm_msg)
        
        if self.home_pose is not None:
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "map"
            pose_msg.pose.position.x = utm_msg.x
            pose_msg.pose.position.y = utm_msg.y
            pose_msg.pose.position.z = 0
            pose_msg.pose.orientation.w = 1.0
            self.pub_pose.publish(pose_msg)

    def odom_callback(self, data):
        if self.home_pose is None:
            self.home_pose = data.pose.pose

if __name__ == '__main__':
    try:
        GpsToUtmConverter()
    except rospy.ROSInterruptException:
        pass
