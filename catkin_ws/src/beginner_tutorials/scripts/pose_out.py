#!usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped

rospy.init_node('pose_publisher')
publisher = rospy.Publisher('/slam_out_pose', PoseStamped, queue_size=1)

rate = rospy.Rate(10)  # 10 Hz

while not rospy.is_shutdown():
    pose_msg = PoseStamped()
    pose_msg.header.frame_id = 'map'
    pose_msg.pose.position.x = 1.0
    pose_msg.pose.position.y = 1.0
    pose_msg.pose.orientation.z = 1.0

    publisher.publish(pose_msg)
    rate.sleep()
