#!usr/bin/env python3
import numpy as np
import math
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import csv
from sensor_msgs.msg import LaserScan

pose = np.zeros(3) # Declare pose as a global variable with initial value [0,0,0]
ld = 0.7
L = 0.30
ESC = 45
stop = 0
csv_path = '/home/kishor/catkin_ws/path/path.csv'

# Initialize an empty list to store the data
path = []

# Open the CSV file and read its contents
with open(csv_path, 'r') as csv_file:
    csv_reader = csv.reader(csv_file)
    
    # Iterate over each row in the CSV file and append it to the data list
    for row in csv_reader:
        path.append(row)
        

# Print the first few rows of the data
#print(data[:10])
path = np.array(path)



def pose_callback(pose_msg):
    global pose # Declare  as a global variable inside the function
    # Callback function to process the received message
    #print("Received pose: x={}, y={}, z={}".format(data.pose.position.x, data.pose.position.y, data.pose.position.z))
    #print("Orientation: x={}, y={}, z={}, w={}".format(data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w))
    pose[0] = pose_msg.pose.position.x
    pose[1] = pose_msg.pose.position.y

def imu_callback(imu_msg):
    global pose 
    #print("Linear acceleration: ({},{},{})".format(data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z))
    #print("Angular velocity: ({},{},{})".format(data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z))
    #print("Orientation: ({},{},{},{})".format(data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w))
    yaw = imu_msg.orientation.z
    yaw = (360-yaw)*np.pi/180
    pose[2] = yaw


def scan_callback(scan_msg):
    global stop
    ranges = np.array(scan_msg.ranges)
    nan_indices = np.isnan(ranges)
    ranges[nan_indices] = 12
    l = len(ranges)
    angles = np.linspace(0, 2*np.pi, l)
    x_indices = np.where(ranges < 1)[0]
    ranges1 = ranges[x_indices]
    angles1 = angles[x_indices]
    x11_indices = np.where((angles1 < 0.5))[0]
    x111_indices = np.where((angles1 > (2*np.pi-0.5)))[0]
    x1_indices = np.concatenate((x11_indices, x111_indices))
    ranges2 = ranges1[x1_indices] 
    angles2 = angles1[x1_indices]
    if len(ranges2)>0:
        stop = 0
    else:
        stop = 1


def speed_controller(ESC,pose,path,stop):
    dx = pose[0] - path[len(path)-1,0].astype(float)
    dy= pose[1] - path[len(path)-1,1].astype(float)
    distances = np.sqrt(dx**2 + dy**2)
    if (distances <= 0.3):
        speed = 0
    else:
        speed = ESC*stop
    return speed

    
def nearest_pose(path, current_pose):
    # path: Nx3 matrix containing the x, y, and heading of the waypoints in the path
    # current_pose: 1x3 vector containing the x, y, and heading of the current pose of the vehicle
    # nearest_pose: 1x3 vector containing the x, y, and heading of the nearest pose on the path
    # nearest_index: index of the nearest pose on the path
    
    # Find the distance from the current pose to each waypoint on the path
    l = len(path)
    cx = path[:,0].astype(float)  # Convert cx to float64 data type
    cy = path[:,1].astype(float)  # Convert cy to float64 data type
    dx = np.zeros(l)
    dy = np.zeros(l)
    distances = np.zeros(l)

    for i in range(l):
        dx[i] = current_pose[0] - cx[i]
        dy[i] = current_pose[1] - cy[i]
        distances[i] = np.sqrt(dx[i]**2 + dy[i]**2)

    return distances

def min_distance(distances):
    min_dist = np.min(distances)
    min_index = np.argmin(distances)
    return min_dist, min_index

def Target_point(min_index, path, pose, ld):
    

    if (min_index >=len(path)-2):
        target_index = len(path)-1
        
    else:

        target_index = min_index + 1
        dist = np.sqrt((float(path[target_index,0])-pose[0])**2 + (float(path[target_index,1])-pose[1])**2)
        while (dist < ld):
            target_index += 1
            dist = np.sqrt((float(path[target_index,0])-pose[0])**2 + (float(path[target_index,1])-pose[1])**2)
    return target_index


def pure_pursuit_controller(target_index, path, pose, ld, L):
    dx = float(path[target_index, 0]) - pose[0]
    dy = float(path[target_index, 1]) - pose[1]
    angle = np.arctan2(dy, dx)
    if angle < 0:
        angle = np.pi + (np.pi + angle)
    else:
        pass
    alpha = angle - pose[2]
    if alpha <= -np.pi:
        alpha = angle + (2*np.pi - pose[2])
    elif alpha >= np.pi:
        alpha = -(2*np.pi - angle + pose[2])
    else:
        pass

    rospy.loginfo(alpha*(180/np.pi))
    k = 2*np.sin(alpha)/ld
    delta = np.arctan(k*L)*180/np.pi
    delta = constraint(-30,30,delta)

    return delta



def constraint(a,b,value):
    if value <= a :
        value = a
    elif value >= b :
        value = b
    else:
        value = value
    return value

        
def main():
    rospy.init_node('pure_pursuit_controller', anonymous=True)
    rospy.Subscriber('/slam_out_pose', PoseStamped, pose_callback)
    rospy.Subscriber('/imu/data', Imu, imu_callback)
    rospy.Subscriber('/scan', LaserScan, scan_callback)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        # Check if both topics have received messages before publishing a new message on the new topic
        if True: #hasattr(pose_callback, 'pose_msg') and hasattr(imu_callback, 'imu_msg'):
            # Create a new PoseStamped message and fill it with data from the latest received messages
            distances = nearest_pose(path, pose)
            min_dist, min_index = min_distance(distances)
            target_index = Target_point(min_index, path, pose, ld)
            delta = pure_pursuit_controller(target_index, path, pose, ld, L)
            velocity = speed_controller(ESC,pose,path,stop)
            
            cmd_vel = Twist()
            cmd_vel.linear.x = velocity
            cmd_vel.angular.z = delta
            pub.publish(cmd_vel)
            #rospy.loginfo()
            rate.sleep()

if __name__ == '__main__':
    try:
        main()
        
    except rospy.ROSInterruptException:
        pass



