#!usr/bin/env python3
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def scan_callback(scan_msg):
    #angles = np.arange(scan_msg.angle_min, scan_msg.angle_max, scan_msg.angle_increment)
    ranges = np.array(scan_msg.ranges)
    nan_indices = np.isnan(ranges)
    ranges[nan_indices] = 12
    l = len(ranges)
    angles = np.linspace(0, 2*np.pi, l)
    #polar_ranges = np.vstack((ranges * np.cos(angles), ranges * np.sin(angles))).T
    avoid_obstacle(ranges,angles)
    #rospy.loginfo(ranges)

def avoid_obstacle(ranges,angles):
    # Find ranges with angles within the specified range
    x_indices = np.where(ranges < 1)[0]
    ranges1 = ranges[x_indices]
    angles1 = angles[x_indices]
    x11_indices = np.where((angles1 < 0.5))[0]
    x111_indices = np.where((angles1 > (2*np.pi-0.5)))[0]
    x1_indices = np.concatenate((x11_indices, x111_indices))
    ranges2 = ranges1[x1_indices] 
    angles2 = angles1[x1_indices] 
    rospy.loginfo(len(ranges2))

    if len(ranges2) > 0:
    
        cmd = Twist()
        cmd.linear.x = 0  
        cmd_pub.publish(cmd)
    else:
        # Generate command to move forward
        cmd = Twist()
        cmd.linear.x = 40  # move at a fixed speed
        cmd_pub.publish(cmd)
    

if __name__ == '__main__':
    rospy.init_node('obstacle_avoidance')
    rospy.Subscriber('/scan', LaserScan, scan_callback)
    cmd_pub = rospy.Publisher('speed', Twist, queue_size=10)
    rospy.spin()

