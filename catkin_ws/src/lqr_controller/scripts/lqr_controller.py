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
velocity = 1 # m/s
Q = np.array([[25,0,0,0],[0,1,0,0],[0,0,30,0],[0,0,0,0.5]])
R = np.array([[10]])
ESC = 41
dt = 1/34.5
stop = 0
csv_path = '/home/kishor/catkin_ws/path/path.csv'

# Initialize an empty list to store the data
path = []
ydot = 0
xdot = 0
w = 0

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
    pose[0] = pose_msg.pose.position.x + 0.15*math.cos(pose[2])
    pose[1] = pose_msg.pose.position.y + 0.15*math.sin(pose[2])

def imu_callback(imu_msg):
    global pose
    global ydot
    global xdot
    global w 
    #print("Linear acceleration: ({},{},{})".format(data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z))
    #print("Angular velocity: ({},{},{})".format(data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z))
    #print("Orientation: ({},{},{},{})".format(data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w))
    yaw = imu_msg.orientation.z
    yaw = (360-yaw)*np.pi/180
    pose[2] = yaw
    w = imu_msg.angular_velocity.z
    ydot = (imu_msg.linear_acceleration.y-0.00196172244419)*dt
    xdot = (imu_msg.linear_acceleration.x-0.000159489629609)*dt

def scan_callback(scan_msg):
    global stop
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
    dx = pose[0] - path[len(path)-1,0].astype(float)
    dy= pose[1] - path[len(path)-1,1].astype(float)
    distances = np.sqrt(dx**2 + dy**2)
    if (distances <= 0.3):
        speed = 0
    else:
        speed = ESC*stop
    return speed


def getCurvature(xRef, yRef):
    # Calculate gradient by the gradient of the X and Y vectors
    DX = np.gradient(xRef)
    D2X = np.gradient(DX)
    DY = np.gradient(yRef)
    D2Y = np.gradient(DY)
    curvature = (DX*D2Y - DY*D2X) / (DX**2 + DY**2)**(3/2)
    return curvature

curvature = getCurvature(path[:,0].astype(float), path[:,1].astype(float))



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



def error_calculator(min_dist, ind, curvature, path, pose):
    ref_curv = curvature[ind]
    theta_ref = float(path[ind, 2])

    theta_pose = float(pose[2])
    theta_e = theta_pose - theta_ref
    if theta_e < -np.pi:
        theta_e = theta_pose + (2*np.pi-theta_ref)
    elif theta_e > np.pi:
        theta_e = -(2*np.pi-theta_pose + theta_ref)

    current_position = np.array([float(pose[0]), float(pose[1])])
    next_waypoint = np.array([float(path[ind+1, 0]), float(path[ind+1, 1])])
    waypoint_vector = next_waypoint - current_position
    normal_vector = np.array([-waypoint_vector[1], waypoint_vector[0]])
    vehicle_heading = np.array([np.cos(pose[2]), np.sin(pose[2])])
    dot_product = np.dot(vehicle_heading, normal_vector)
    if dot_product > 0:
        cross_track_error = +1  # vehicle is to the right of the line
    else:
        cross_track_error = -1  # vehicle is to the left of the line

    min_dist = min_dist * cross_track_error

    return min_dist, theta_e, ref_curv


def states(velocity, ref_curv, w, theta_e, min_dist, ydot):
    # rate change of heading error
    e2dot = w - velocity*ref_curv

    e2 = theta_e
        
    # cross track error
    e1 = min_dist
    e1dot = ydot + velocity * e2

    return e1, e1dot, e2, e2dot

def lqr_own(A, B, Q, R):
    p1, p2 = Q.shape
    p = np.vstack((
        np.hstack((-A.T, -Q)),
        np.hstack((-B @ np.linalg.inv(R) @ B.T, A))
    ))

    d, v = np.linalg.eig(p)
    n = np.argsort(np.real(d))
    v = v[:, n]
  

    phis = v[:, :p1]
    phis1 = phis[:p1, :]
    phis2 = phis[p1:, :]


    K = np.real(np.linalg.inv(R) @ B.T @ phis1 @ np.linalg.inv(phis2))
    return K


def steering_angle(Q, R, Vx, e1, e1_dot, e2, e2_dot):
    # defining vehicle parameters
    a = 0.330/2     # m ,distance from CoG to front axel
    b = 0.330/2     # m ,distance from CoG to rear axel
    Iz = 5*0.1**2  # kg-m^2 ,Yaw rate of inertia around CoG
    m = 5     # kg ,mass of the vehicle 
    Cf = 8354   # N/rad ,cornering stiffness of the front axel
    Cr = 8310   # N/rad ,cornering stiffness of the rear axel
    w = 0.25   # m ,vehicle width
    lf = 0.330/2    # m ,CoG to front end 
    lr = 0.330/2    # m ,CoG to rear end

    # state space and control inpute's coefficient matrix
    A = np.array([
        [0, 1, 0, 0],
        [0, -(2*Cf+2*Cr)/(m*Vx), (2*Cf+2*Cf)/(m), (-2*Cf*lf+2*Cf*lr)/(m*Vx)],
        [0, 0, 0, 1],
        [0, -(2*Cf*lf-2*Cf*lr)/(Iz*Vx), (2*Cf*lf-2*Cf*lr)/(Iz), -(2*Cf*lf**2+2*Cf*lr**2)/(Iz*Vx)]
    ])

    B1 = np.array([
        [0],
        [((-2*Cf+Cf)/(m*Vx))-Vx],
        [0],
        [-(2*Cf*lf**2+2*Cf*lr**2)/(Iz*Vx)]
    ])

    B = np.array([
        [0],
        [2*Cf/m],
        [0],
        [2*Cf*lf/Iz]
    ])

    B2 = np.array([[0], [0], [0], [1]])

    # calculating gain matrix
    K = lqr_own(A,B,Q,R)

    # state space matrix
    X = np.array([e1, e1_dot, e2, e2_dot])

    # steering angle
    delta = (- K @ X)*180/np.pi
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
    rospy.init_node('lqr_controller', anonymous=True)
    rospy.Subscriber('/slam_out_pose', PoseStamped, pose_callback)
    rospy.Subscriber('/imu/data', Imu, imu_callback)
    rospy.Subscriber('/scan', LaserScan, scan_callback)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(30)
    global velocity

    while not rospy.is_shutdown():
        # Check if both topics have received messages before publishing a new message on the new topic
        if True: #hasattr(pose_callback, 'pose_msg') and hasattr(imu_callback, 'imu_msg'):
            # Create a new PoseStamped message and fill it with data from the latest received messages
            distances = nearest_pose(path, pose)
            min_dist, ind = min_distance(distances)
            min_dist, theta_e, ref_curv= error_calculator(min_dist, ind, curvature, path, pose)
            e1, e1_dot, e2, e2_dot = states(velocity, ref_curv, w, theta_e, min_dist, ydot)
            delta = steering_angle(Q, R, velocity, e1, e1_dot, e2, e2_dot)
            speed = speed_controller(ESC,pose,path,stop)

            cmd_vel = Twist()
            cmd_vel.linear.x = speed
            cmd_vel.angular.z = delta
            pub.publish(cmd_vel)
            rospy.loginfo([speed,delta])
            rate.sleep()

if __name__ == '__main__':
    try:
        main()
        
    except rospy.ROSInterruptException:
        pass