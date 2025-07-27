#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

class OdometryGenerator():
    def __init__(self):
        rospy.init_node('odometry_generator', anonymous=True)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.path_sub = rospy.Subscriber('/path', Path, self.path_callback)
        self.current_pose = None
        self.previous_pose = None
        self.last_time = rospy.Time.now()

    def path_callback(self, path_msg):
        if not path_msg.poses:
            return
        self.current_pose = path_msg.poses[-1].pose
        if self.previous_pose is None:
            self.previous_pose = self.current_pose
        self.publish_odometry()

    def publish_odometry(self):
        current_time = rospy.Time.now()
        elapsed_time = (current_time - self.last_time).to_sec()

        # Get position from current pose
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        z = self.current_pose.position.z

        # Convert orientation from quaternion to euler angles
        orientation = self.current_pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w])

        # Calculate linear and angular velocities
        delta_x = x - self.previous_pose.position.x
        delta_y = y - self.previous_pose.position.y
        delta_z = z - self.previous_pose.position.z
        linear_velocity = ((delta_x**2 + delta_y**2 + delta_z**2)**0.5) / elapsed_time

        delta_yaw = yaw - self.previous_yaw
        angular_velocity = delta_yaw / elapsed_time

        # Publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = z
        odom_msg.pose.pose.orientation = orientation
        odom_msg.twist.twist.linear.x = linear_velocity
        odom_msg.twist.twist.angular.z = angular_velocity

        self.odom_pub.publish(odom_msg)

        # Update previous pose and time
        self.previous_pose = self.current_pose
        self.previous_yaw = yaw
        self.last_time = current_time

if __name__ == '__main__':
    try:
        OdometryGenerator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
