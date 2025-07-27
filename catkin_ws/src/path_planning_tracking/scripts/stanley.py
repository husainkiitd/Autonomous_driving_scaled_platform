#!usr/bin/env python3
import numpy as np
import math
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
import csv
from custom_msgs.msg import custom

pose = np.zeros(3) # Declare pose as a global variable with initial value [0,0,0]
K = 2.75
Ve = 0
Vx = 1
ESC = 40
stop = 0
csv_path = '/home/kishor/catkin_ws/path/path.csv'


Tracking = False

# Initialize an empty list to store the data
path = []
print(len(path))
goal_pose = np.zeros(3)

def path_callback(path_msgs):
    global path
    global Tracking
    if len(path)==0:
        with open(csv_path, 'r') as csv_file:
            csv_reader = csv.reader(csv_file)
    
            # Iterate over each row in the CSV file and append it to the data list
            for row in csv_reader:
                path.append(row)
        path = np.array(path)
        Tracking = True
        rospy.loginfo("Path Recieved  .....Tracking Started")
    else:
        path1 = []
        with open(csv_path, 'r') as csv_file:
            csv_reader = csv.reader(csv_file)
    
            # Iterate over each row in the CSV file and append it to the data list
            for row in csv_reader:
                path1.append(row)
        path1 = np.array(path1)
        path = path1      
        Tracking = True
        rospy.loginfo("Path Recieved  .....Tracking Started")
    
    


def pose_callback(pose_msg):
    global pose # Declare  as a global variable inside the function
    # Callback function to process the received message
    #print("Received pose: x={}, y={}, z={}".format(data.pose.position.x, data.pose.position.y, data.pose.position.z))
    #print("Orientation: x={}, y={}, z={}, w={}".format(data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w))
    pose[0] = pose_msg.pose.position.x + 0.3*math.cos(pose[2])
    pose[1] = pose_msg.pose.position.y + 0.3*math.sin(pose[2])

def imu_callback(imu_msg):
    global pose 
    #print("Linear acceleration: ({},{},{})".format(data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z))
    #print("Angular velocity: ({},{},{})".format(data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z))
    #print("Orientation: ({},{},{},{})".format(data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w))
    yaw = imu_msg.orientation.z
    yaw = (360-yaw)*np.pi/180
    pose[2] =yaw



def angle(z):
    if (z<=0):
        z = 2+z
    
    z = z*np.pi
    return z


def scan_callback(scan_msg):
    global stop
    global Tracking
    global goal_pose
    #angles = np.arange(scan_msg.angle_min, scan_msg.angle_max, scan_msg.angle_increment)
    ranges = np.array(scan_msg.ranges)
    nan_indices = np.isnan(ranges)
    ranges[nan_indices] = 12
    l = len(ranges)
    angles = np.linspace(0, 2*np.pi, l)
    #polar_ranges = np.vstack((ranges * np.cos(angles), ranges * np.sin(angles))).T
    #rospy.loginfo(ranges)
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
    global Tracking
    dx = pose[0] - path[len(path)-1,0].astype(float)
    dy= pose[1] - path[len(path)-1,1].astype(float)
    distances = np.sqrt(dx**2 + dy**2)
    if (distances <= 0.3):
        speed = 0
        Tracking = False
        rospy.loginfo("....Reached at destination.... ")
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



def error_calculator(mind, ind, path, pose, esc):
    if ind >= len(path)-2:
        ind = ind-1
    else:
        pass

    theta_pose = float(pose[2])
    theta_ref = float(path[ind, 2])  # Convert theta_ref to float type
    theta_e = theta_ref - theta_pose
    if theta_e <= -np.pi:
        theta_e = theta_ref + (2*np.pi-theta_pose)
    elif theta_e >= np.pi:
        theta_e = -(2*np.pi-theta_ref + theta_pose)

    current_position = np.array([float(pose[0]), float(pose[1])])
    next_waypoint = np.array([float(path[ind+1, 0]), float(path[ind+1, 1])])
    waypoint_vector = next_waypoint - current_position
    normal_vector = np.array([-waypoint_vector[1], waypoint_vector[0]])
    vehicle_heading = np.array([np.cos(pose[2]), np.sin(pose[2])])
    dot_product = np.dot(vehicle_heading, normal_vector)
    if dot_product > 0:
        cross_track_error = -1  # vehicle is to the right of the line
    else:
        cross_track_error = +1  # vehicle is to the left of the line

    min_d = mind * cross_track_error

    Esc = speed_controller(esc,pose,path,stop)
    return min_d, theta_e, Esc



def LateralControllerStanley(K, Ve, Vx, mind, theta_e):
    delta = (theta_e + math.atan(K * mind / (Ve + Vx)))*180/np.pi
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
    rospy.init_node('stanley_controller', anonymous=True)
    rospy.Subscriber('/slam_out_pose', PoseStamped, pose_callback)
    rospy.Subscriber('/imu/data', Imu, imu_callback)
    rospy.Subscriber('/scan', LaserScan, scan_callback)
    rospy.Subscriber('globalpath' ,custom, path_callback)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(30)
    rospy.loginfo("Stanley Contriller Srarted")



    while not rospy.is_shutdown():
        global Tracking
    
        if Tracking:             
            distances = nearest_pose(path, pose)
            min_dist, ind = min_distance(distances)
            min_d, theta_e, esc = error_calculator(min_dist, ind, path, pose, ESC)
            delta = LateralControllerStanley(K, Ve, Vx, min_d, theta_e,)
            cmd_vel = Twist()
            cmd_vel.linear.x = esc
            cmd_vel.angular.z = delta
            pub.publish(cmd_vel)
            #rospy.loginfo([min_d,theta_e])
            #rospy.loginfo("...running...")
            rate.sleep()

        else:
            cmd_vel = Twist()
            cmd_vel.linear.x = 0
            cmd_vel.angular.z = 0
            pub.publish(cmd_vel)
            #rospy.loginfo("....stoped.....")
            rate.sleep()
            

if __name__ == '__main__':
    try:
        main()
        
    except rospy.ROSInterruptException:
        
        pass



