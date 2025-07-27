#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from nav_msgs.msg import Odometry
import tf.transformations as transformations
import pyrealsense2 as rs
import cv2




class RealSenseCamera:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('realsense_camera')

        # Initialize RealSense pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        self.pipeline_profile = self.config.resolve(self.pipeline_wrapper)
        self.device = self.pipeline_profile.get_device()
        self.device_product_line = str(self.device.get_info(rs.camera_info.product_line))
       
        # Initialize ROS publisher
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)

    def run(self):
        self.pipeline.start(self.config)
        while not rospy.is_shutdown():
            # Wait for a new frame
            frames = self.pipeline.wait_for_frames()
            pose = frames.get_pose_frame()

            # Create the Odometry message
            odom_msg = Odometry()
            odom_msg.header.stamp = rospy.Time.now()
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_link'

            # Set the position
            # Set the position
            translation = pose.get_pose_data().translation
            odom_msg.pose.pose.position.x = translation.x
            odom_msg.pose.pose.position.y = translation.y
            odom_msg.pose.pose.position.z = translation.z


            # Set the orientation
            rot = pose.get_rotation_matrix()
            quat = transformations.quaternion_from_matrix(rot)
            odom_msg.pose.pose.orientation.x = quat[0]
            odom_msg.pose.pose.orientation.y = quat[1]
            odom_msg.pose.pose.orientation.z = quat[2]
            odom_msg.pose.pose.orientation.w = quat[3]

            # Publish the Odometry message
            self.odom_pub.publish(odom_msg)


if __name__ == '__main__':
    try:
        camera = RealSenseCamera()
        camera.run()
    except rospy.ROSInterruptException:
        pass
