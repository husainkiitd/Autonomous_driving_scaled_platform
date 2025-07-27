#!usr/bin/env python3
import cv2
import os
import heapq
import math
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
import numpy as np
from scipy.spatial import cKDTree
from dynamic_programming_heuristic import calc_distance_heuristic
import reeds_shepp_path_planning as rs
from car import move, check_car_collision, MAX_STEER, WB, plot_car, BUBBLE_R
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from custom_msgs.msg import custom
from sensor_msgs.msg import Imu
import tf


XY_GRID_RESOLUTION = 0.050  # [m]
YAW_GRID_RESOLUTION = np.deg2rad(0.524)  # [rad]
MOTION_RESOLUTION = 0.1  # [m] path interpolate resolution
N_STEER = 10  # number of steer command

SB_COST = 100.0  # switch back penalty cost
BACK_COST = 1000.0  # backward penalty cost
STEER_CHANGE_COST = 5.0  # steer angle change penalty cost
STEER_COST = 5.0 # steer angle change penalty cost
H_COST = 5.0  # Heuristic cost

show_animation = True

start_pose = np.zeros(3)
goal_pose = np.zeros(3)

#####################################################

# Read the PGM file using OpenCV
pgm_file = "my_map.pgm"
#pgm_file = map_name
img = cv2.imread(pgm_file, cv2.IMREAD_GRAYSCALE)

# Set the threshold values
occupied_thresh = 0.65
free_thresh = 0.196

# Convert the pixel values to binary values
binary_img = np.zeros_like(img)
binary_img[img >= occupied_thresh] = 1
binary_img[img <= free_thresh] = 0

# Set the origin and resolution
resolution = 0.050000
origin = [-51.224998, -51.224998]

# Create the occupancy grid map
map_width = binary_img.shape[1]
map_height = binary_img.shape[0]
occupancy_map = np.zeros((map_height, map_width))
#obstacleList = []
ox = []
oy = []

for i in range(map_height):
    for j in range(map_width):
        x = origin[0] + (j + 0.5) * resolution
        y = origin[1] + (map_height - i - 0.5) * resolution
        occupancy_map[i, j] = binary_img[i, j]
        if binary_img[i, j] == 0:
            ox.append(x)
            oy.append(y)
            #obstacleList.append((x, y, 0.2)) # assuming robot radius is 0.8 meters

# Save the occupancy grid map as a text file
#np.savetxt("occupancy_map.txt", occupancy_map, fmt="%d")

# Visualize the occupancy grid map
"""
plt.imshow(occupancy_map, cmap="gray")
plt.show()
print(len(ox))
"""


#####################################################


def angle(z):
    if (z<=0):
        z = 2+z
    
    z = z*np.pi
    return z



def start_pose_callback(pose_msg):
    global start_pose # Declare  as a global variable inside the function
    # Callback function to process the received message
    #print("Received pose: x={}, y={}, z={}".format(data.pose.position.x, data.pose.position.y, data.pose.position.z))
    #print("Orientation: x={}, y={}, z={}, w={}".format(data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w))
    start_pose[0] = pose_msg.pose.position.x + 0.3*math.cos(start_pose[2])
    start_pose[1] = pose_msg.pose.position.y + 0.3*math.sin(start_pose[2])

def imu_callback(imu_msg):
    global start_pose 
    #print("Linear acceleration: ({},{},{})".format(data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z))
    #print("Angular velocity: ({},{},{})".format(data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z))
    #print("Orientation: ({},{},{},{})".format(data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w))
    yaw = imu_msg.orientation.z
    yaw = (360-yaw)*np.pi/180
    start_pose[2] =yaw


def goal_pose_callback(pose_msg):
    global goal_pose
    global oxx
    global oyy 
    goal_pose[0] = pose_msg.pose.position.x
    goal_pose[1] = pose_msg.pose.position.y

    # Extract the orientation of the goal pose from the message
    quat = [pose_msg.pose.orientation.x, pose_msg.pose.orientation.y,
            pose_msg.pose.orientation.z, pose_msg.pose.orientation.w]

    # Convert the quaternion to Euler angles
    euler = tf.transformations.euler_from_quaternion(quat)
    # Extract the roll, pitch, and yaw angles from the Euler angles
    roll, pitch, yaw_z = euler  
    
    #goal_pose[2] = angle(pose_msg.pose.orientation.z)
    goal_pose[2] = yaw_z

    rospy.loginfo(goal_pose)
    # Set Initial parameters
    start = [start_pose[0], start_pose[1], start_pose[2]]
    goal = [goal_pose[0], goal_pose[1], goal_pose[2]]

    print("start : ", start)
    print("goal : ", goal)

    if show_animation:
        plt.plot(ox, oy, ".k")
        rs.plot_arrow(start[0], start[1], start[2], fc='g')
        rs.plot_arrow(goal[0], goal[1], goal[2])
        plt.grid(True)
        plt.axis("equal")

    path = hybrid_a_star_planning(
        start, goal, ox, oy, XY_GRID_RESOLUTION, YAW_GRID_RESOLUTION)
    print(path)
    x = path.x_list
    y = path.y_list
    yaw = path.yaw_list
    

    path_Smoothing(x,y)



    if show_animation:
        for i_x, i_y, i_yaw in zip(x, y, yaw):
            plt.cla()
            plt.plot(ox, oy, ".k")
            plt.plot(x, y, "-r", label="Hybrid A* path")
            plt.grid(True)
            plt.axis("equal")
            plot_car(i_x, i_y, i_yaw)
            plt.pause(0.1)

def path_Smoothing(x,y):
    refPose = np.vstack((x, y)).T
    xRef = refPose[:,0]
    yRef = refPose[:,1]

    # calculate distance vector
    distancematrix = np.zeros((len(refPose), len(refPose)))
    for i in range(len(refPose)):
        for j in range(len(refPose)):
            distancematrix[i][j] = np.linalg.norm(refPose[i] - refPose[j])
    distancesteps = np.zeros(len(refPose)-1)
    for i in range(1, len(refPose)):
        distancesteps[i-1] = distancematrix[i][i-1]
    totalDistance = np.sum(distancesteps) # Total distance travelled
    distbp = np.cumsum(np.concatenate(([0], distancesteps))) # Distance for each waypoint
    gradbp = np.linspace(0, distbp[-1], 1000) # Linearize distance

    # linearize X and Y vectors based on distance
    xRef2 = interp1d(distbp, xRef)(gradbp)
    yRef2 = interp1d(distbp, yRef)(gradbp)
    yRef2s = interp1d(gradbp, yRef2, kind='cubic')(gradbp) # smooth waypoints
    xRef2s = interp1d(gradbp, xRef2, kind='cubic')(gradbp) # smooth waypoints


    thetaRef = np.zeros((len(gradbp), 1))
    for i in range(1, len(gradbp)):
        thetaRef[i, 0] = np.arctan2((yRef2[i]-yRef2[i-1]), (xRef2[i]-xRef2[i-1]))
        if (thetaRef[i, 0] < 0):
            thetaRef[i, 0] = np.pi + (np.pi + thetaRef[i, 0])
        else:
            pass
    thetaRefs = np.interp(gradbp, gradbp, thetaRef.flatten(), period=totalDistance)

    
    Refpaths = np.column_stack((xRef2s, yRef2s, thetaRefs)).T
    file_path = os.path.expanduser("~/catkin_ws/path/path.csv")
    np.savetxt(file_path, Refpaths.T, delimiter=",")
    
    pub = rospy.Publisher('globalpath', custom, queue_size=10)
    msg = custom()
    """
    msg.x = xRef2s.tolist()
    msg.y = yRef2s.tolist()
    msg.theta = thetaRefs.tolist()
    """
    pub.publish(msg)
    rospy.loginfo(msg)
    rospy.loginfo('.....................path.....................')   




class Node:

    def __init__(self, x_ind, y_ind, yaw_ind, direction,
                 x_list, y_list, yaw_list, directions,
                 steer=0.0, parent_index=None, cost=None):
        self.x_index = x_ind
        self.y_index = y_ind
        self.yaw_index = yaw_ind
        self.direction = direction
        self.x_list = x_list
        self.y_list = y_list
        self.yaw_list = yaw_list
        self.directions = directions
        self.steer = steer
        self.parent_index = parent_index
        self.cost = cost


class Path:

    def __init__(self, x_list, y_list, yaw_list, direction_list, cost):
        self.x_list = x_list
        self.y_list = y_list
        self.yaw_list = yaw_list
        self.direction_list = direction_list
        self.cost = cost


class Config:

    def __init__(self, ox, oy, xy_resolution, yaw_resolution):
        min_x_m = min(ox)
        min_y_m = min(oy)
        max_x_m = max(ox)
        max_y_m = max(oy)

        ox.append(min_x_m)
        oy.append(min_y_m)
        ox.append(max_x_m)
        oy.append(max_y_m)

        self.min_x = round(min_x_m / xy_resolution)
        self.min_y = round(min_y_m / xy_resolution)
        self.max_x = round(max_x_m / xy_resolution)
        self.max_y = round(max_y_m / xy_resolution)

        self.x_w = round(self.max_x - self.min_x)
        self.y_w = round(self.max_y - self.min_y)

        self.min_yaw = round(- math.pi / yaw_resolution) - 1
        self.max_yaw = round(math.pi / yaw_resolution)
        self.yaw_w = round(self.max_yaw - self.min_yaw)


def calc_motion_inputs():
    for steer in np.concatenate((np.linspace(-MAX_STEER, MAX_STEER,
                                             N_STEER), [0.0])):
        for d in [1, -1]:
            yield [steer, d]


def get_neighbors(current, config, ox, oy, kd_tree):
    for steer, d in calc_motion_inputs():
        node = calc_next_node(current, steer, d, config, ox, oy, kd_tree)
        if node and verify_index(node, config):
            yield node


def calc_next_node(current, steer, direction, config, ox, oy, kd_tree):
    x, y, yaw = current.x_list[-1], current.y_list[-1], current.yaw_list[-1]

    arc_l = XY_GRID_RESOLUTION * 1.5
    x_list, y_list, yaw_list = [], [], []
    for _ in np.arange(0, arc_l, MOTION_RESOLUTION):
        x, y, yaw = move(x, y, yaw, MOTION_RESOLUTION * direction, steer)
        x_list.append(x)
        y_list.append(y)
        yaw_list.append(yaw)

    if not check_car_collision(x_list, y_list, yaw_list, ox, oy, kd_tree):
        return None

    d = direction == 1
    x_ind = round(x / XY_GRID_RESOLUTION)
    y_ind = round(y / XY_GRID_RESOLUTION)
    yaw_ind = round(yaw / YAW_GRID_RESOLUTION)

    added_cost = 0.0

    if d != current.direction:
        added_cost += SB_COST

    # steer penalty
    added_cost += STEER_COST * abs(steer)

    # steer change penalty
    added_cost += STEER_CHANGE_COST * abs(current.steer - steer)

    cost = current.cost + added_cost + arc_l

    node = Node(x_ind, y_ind, yaw_ind, d, x_list,
                y_list, yaw_list, [d],
                parent_index=calc_index(current, config),
                cost=cost, steer=steer)

    return node


def is_same_grid(n1, n2):
    if n1.x_index == n2.x_index \
            and n1.y_index == n2.y_index \
            and n1.yaw_index == n2.yaw_index:
        return True
    return False


def analytic_expansion(current, goal, ox, oy, kd_tree):
    start_x = current.x_list[-1]
    start_y = current.y_list[-1]
    start_yaw = current.yaw_list[-1]

    goal_x = goal.x_list[-1]
    goal_y = goal.y_list[-1]
    goal_yaw = goal.yaw_list[-1]

    max_curvature = math.tan(MAX_STEER) / WB
    paths = rs.calc_paths(start_x, start_y, start_yaw,
                          goal_x, goal_y, goal_yaw,
                          max_curvature, step_size=MOTION_RESOLUTION)

    if not paths:
        return None

    best_path, best = None, None

    for path in paths:
        if check_car_collision(path.x, path.y, path.yaw, ox, oy, kd_tree):
            cost = calc_rs_path_cost(path)
            if not best or best > cost:
                best = cost
                best_path = path

    return best_path


def update_node_with_analytic_expansion(current, goal,
                                        c, ox, oy, kd_tree):
    path = analytic_expansion(current, goal, ox, oy, kd_tree)

    if path:
        if show_animation:
            plt.plot(path.x, path.y)
        f_x = path.x[1:]
        f_y = path.y[1:]
        f_yaw = path.yaw[1:]

        f_cost = current.cost + calc_rs_path_cost(path)
        f_parent_index = calc_index(current, c)

        fd = []
        for d in path.directions[1:]:
            fd.append(d >= 0)

        f_steer = 0.0
        f_path = Node(current.x_index, current.y_index, current.yaw_index,
                      current.direction, f_x, f_y, f_yaw, fd,
                      cost=f_cost, parent_index=f_parent_index, steer=f_steer)
        return True, f_path

    return False, None


def calc_rs_path_cost(reed_shepp_path):
    cost = 0.0
    for length in reed_shepp_path.lengths:
        if length >= 0:  # forward
            cost += length
        else:  # back
            cost += abs(length) * BACK_COST

    # switch back penalty
    for i in range(len(reed_shepp_path.lengths) - 1):
        # switch back
        if reed_shepp_path.lengths[i] * reed_shepp_path.lengths[i + 1] < 0.0:
            cost += SB_COST

    # steer penalty
    for course_type in reed_shepp_path.ctypes:
        if course_type != "S":  # curve
            cost += STEER_COST * abs(MAX_STEER)

    # ==steer change penalty
    # calc steer profile
    n_ctypes = len(reed_shepp_path.ctypes)
    u_list = [0.0] * n_ctypes
    for i in range(n_ctypes):
        if reed_shepp_path.ctypes[i] == "R":
            u_list[i] = - MAX_STEER
        elif reed_shepp_path.ctypes[i] == "L":
            u_list[i] = MAX_STEER

    for i in range(len(reed_shepp_path.ctypes) - 1):
        cost += STEER_CHANGE_COST * abs(u_list[i + 1] - u_list[i])

    return cost


def hybrid_a_star_planning(start, goal, ox, oy, xy_resolution, yaw_resolution):
    """
    start: start node
    goal: goal node
    ox: x position list of Obstacles [m]
    oy: y position list of Obstacles [m]
    xy_resolution: grid resolution [m]
    yaw_resolution: yaw angle resolution [rad]
    """

    start[2], goal[2] = rs.pi_2_pi(start[2]), rs.pi_2_pi(goal[2])
    tox, toy = ox[:], oy[:]

    obstacle_kd_tree = cKDTree(np.vstack((tox, toy)).T)

    config = Config(tox, toy, xy_resolution, yaw_resolution)

    start_node = Node(round(start[0] / xy_resolution),
                      round(start[1] / xy_resolution),
                      round(start[2] / yaw_resolution), True,
                      [start[0]], [start[1]], [start[2]], [True], cost=0)
    goal_node = Node(round(goal[0] / xy_resolution),
                     round(goal[1] / xy_resolution),
                     round(goal[2] / yaw_resolution), True,
                     [goal[0]], [goal[1]], [goal[2]], [True])

    openList, closedList = {}, {}

    h_dp = calc_distance_heuristic(
        goal_node.x_list[-1], goal_node.y_list[-1],
        ox, oy, xy_resolution, BUBBLE_R)

    pq = []
    openList[calc_index(start_node, config)] = start_node
    heapq.heappush(pq, (calc_cost(start_node, h_dp, config),
                        calc_index(start_node, config)))
    final_path = None

    while True:
        if not openList:
            print("Error: Cannot find path, No open set")
            return [], [], []

        cost, c_id = heapq.heappop(pq)
        if c_id in openList:
            current = openList.pop(c_id)
            closedList[c_id] = current
        else:
            continue

        if show_animation:  # pragma: no cover
            plt.plot(current.x_list[-1], current.y_list[-1], "xc")
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            if len(closedList.keys()) % 10 == 0:
                plt.pause(0.001)

        is_updated, final_path = update_node_with_analytic_expansion(
            current, goal_node, config, ox, oy, obstacle_kd_tree)

        if is_updated:
            print("path found")
            break

        for neighbor in get_neighbors(current, config, ox, oy,
                                      obstacle_kd_tree):
            neighbor_index = calc_index(neighbor, config)
            if neighbor_index in closedList:
                continue
            if neighbor not in openList \
                    or openList[neighbor_index].cost > neighbor.cost:
                heapq.heappush(
                    pq, (calc_cost(neighbor, h_dp, config),
                         neighbor_index))
                openList[neighbor_index] = neighbor

    path = get_final_path(closedList, final_path)
    return path


def calc_cost(n, h_dp, c):
    ind = (n.y_index - c.min_y) * c.x_w + (n.x_index - c.min_x)
    if ind not in h_dp:
        return n.cost + 999999999  # collision cost
    return n.cost + H_COST * h_dp[ind].cost


def get_final_path(closed, goal_node):
    reversed_x, reversed_y, reversed_yaw = \
        list(reversed(goal_node.x_list)), list(reversed(goal_node.y_list)), \
        list(reversed(goal_node.yaw_list))
    direction = list(reversed(goal_node.directions))
    nid = goal_node.parent_index
    final_cost = goal_node.cost

    while nid:
        n = closed[nid]
        reversed_x.extend(list(reversed(n.x_list)))
        reversed_y.extend(list(reversed(n.y_list)))
        reversed_yaw.extend(list(reversed(n.yaw_list)))
        direction.extend(list(reversed(n.directions)))

        nid = n.parent_index

    reversed_x = list(reversed(reversed_x))
    reversed_y = list(reversed(reversed_y))
    reversed_yaw = list(reversed(reversed_yaw))
    direction = list(reversed(direction))

    # adjust first direction
    direction[0] = direction[1]

    path = Path(reversed_x, reversed_y, reversed_yaw, direction, final_cost)

    return path


def verify_index(node, c):
    x_ind, y_ind = node.x_index, node.y_index
    if c.min_x <= x_ind <= c.max_x and c.min_y <= y_ind <= c.max_y:
        return True

    return False


def calc_index(node, c):
    ind = (node.yaw_index - c.min_yaw) * c.x_w * c.y_w + \
          (node.y_index - c.min_y) * c.x_w + (node.x_index - c.min_x)

    if ind <= 0:
        print("Error(calc_index):", ind)

    return ind



def main():
    print("Start Hybrid A* planning")
    rospy.init_node('hybrid_a_star_path_planner', anonymous=True)
    #rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, start_pose_callback)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, goal_pose_callback)
    rospy.Subscriber('/slam_out_pose', PoseStamped, start_pose_callback)
    rospy.Subscriber('/imu/data', Imu, imu_callback) 
    rospy.spin()    
    print(__file__ + " done!!")


if __name__ == '__main__':
    try:
        main()
        
    except rospy.ROSInterruptException:
        pass
