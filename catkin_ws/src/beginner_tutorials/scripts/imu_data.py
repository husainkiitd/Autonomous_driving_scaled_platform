#!usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu

rospy.init_node('imu_publisher')
publisher = rospy.Publisher('/imu/data', Imu, queue_size=10)

rate = rospy.Rate(25.5)  # 25.5 Hz

while not rospy.is_shutdown():
    imu_msg = Imu()
    imu_msg.header.frame_id = 'imu_frame'
    imu_msg.linear_acceleration.x = 1.0
    imu_msg.linear_acceleration.y = 2.0
    imu_msg.linear_acceleration.z = 3.0
    imu_msg.angular_velocity.x = 0.1
    imu_msg.angular_velocity.y = 0.2
    imu_msg.angular_velocity.z = 0.1
    imu_msg.orientation.w = 1.0

    publisher.publish(imu_msg)
    rate.sleep()
