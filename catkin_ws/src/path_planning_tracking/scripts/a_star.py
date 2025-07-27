
#!usr/bin/env python3
import math
import os
import cv2
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from custom_msgs.msg import custom
from sensor_msgs.msg import Imu



show_animation = False


start_pose = np.zeros(3)
goal_pose = np.zeros(3)

#####################################################

# Read the PGM file using OpenCV
pgm_file = "my_map1.pgm"
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
if show_animation:

   plt.imshow(occupancy_map, cmap="gray")
   plt.show()
   print(len(ox))



#####################################################


def start_pose_callback(pose_msg):
    global start_pose # Declare  as a global variable inside the function
    # Callback function to process the received message
    #print("Received pose: x={}, y={}, z={}".format(data.pose.position.x, data.pose.position.y, data.pose.position.z))
    #print("Orientation: x={}, y={}, z={}, w={}".format(data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w))
    start_pose[0] = pose_msg.pose.position.x
    start_pose[1] = pose_msg.pose.position.y

def imu_callback(imu_msg):
    global start_pose 
    #print("Linear acceleration: ({},{},{})".format(data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z))
    #print("Angular velocity: ({},{},{})".format(data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z))
    #print("Orientation: ({},{},{},{})".format(data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w))
    yaw = imu_msg.orientation.z
    yaw = (360-yaw)*np.pi/180
    start_pose[2] =yaw


def angle(z):
    if (z<=0):
        z = 2+z    
    z = z*np.pi
    return z

def goal_pose_callback(pose_msg):
    global goal_pose

    goal_pose[0] = pose_msg.pose.position.x
    goal_pose[1] = pose_msg.pose.position.y
    goal_pose[2] = angle(pose_msg.pose.orientation.z)
    # Set Initial parameters
    start = [start_pose[0], start_pose[1], start_pose[2]]
    goal = [goal_pose[0], goal_pose[1], goal_pose[2]]

    print("start : ", start)
    print("goal : ", goal)

    # start and goal position
    sx = start_pose[0] # [m]
    sy = start_pose[1]  # [m]
    gx = goal_pose[0] # [m]
    gy = goal_pose[1]  # [m]
    grid_size = 0.05  # [m]
    robot_radius = 1.0  # [m]

    if show_animation:  # pragma: no cover
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "og")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")

    a_star = AStarPlanner(ox, oy, grid_size, robot_radius)
    rx, ry = a_star.planning(sx, sy, gx, gy)
    print(len(rx))
    path_Smoothing(np.flip(rx),np.flip(ry))
    #if show_animation:  # pragma: no cover
    plt.plot(rx, ry, "-r")
    plt.pause(0.001)
    plt.show()



class AStarPlanner:

    def __init__(self, ox, oy, resolution, rr):
        """
        Initialize grid map for a star planning

        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m]
        rr: robot radius[m]
        """

        self.resolution = resolution
        self.rr = rr
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.motion = self.get_motion_model()
        self.calc_obstacle_map(ox, oy)

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):
        """
        A star path search

        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        while True:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node,
                                                                     open_set[
                                                                         o]))
            current = open_set[c_id]

            # show graph
            if show_animation:  # pragma: no cover
                plt.plot(self.calc_grid_position(current.x, self.min_x),
                         self.calc_grid_position(current.y, self.min_y), "xc")
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event',
                                             lambda event: [exit(
                                                 0) if event.key == 'escape' else None])
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)

        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, min_position):
        """
        calc grid position

        :param index:
        :param min_position:
        :return:
        """
        pos = index * self.resolution + min_position
        return pos

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.max_x:
            return False
        elif py >= self.max_y:
            return False

        # collision check
        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):

        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))
        print("min_x:", self.min_x)
        print("min_y:", self.min_y)
        print("max_x:", self.max_x)
        print("max_y:", self.max_y)

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        print("x_width:", self.x_width)
        print("y_width:", self.y_width)

        # obstacle map generation
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        self.obstacle_map[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion
    



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


def main():
    print(__file__ + " start!!")
    rospy.init_node('a_star_path_planner', anonymous=True)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, goal_pose_callback)
    rospy.Subscriber('/slam_out_pose', PoseStamped, start_pose_callback)
    rospy.Subscriber('/imu/data', Imu, imu_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
        
    except rospy.ROSInterruptException:
        pass