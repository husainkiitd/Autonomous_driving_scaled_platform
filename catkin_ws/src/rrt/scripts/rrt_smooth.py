#!/usr/bin/env python3
import cv2
import os
import math
import random
import rospy
import numpy as np
from custom_msgs.msg import custom
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from scipy.interpolate import CubicSpline

show_animation = True
# Initialize the ROS node
rospy.init_node('my_talker', anonymous=True)

# Create a publisher that sends messages on the 'my_topic' topic using the custom message type
pub = rospy.Publisher('my_topic', custom, queue_size=1)

# Set the loop rate (in Hz)
rate = rospy.Rate(25.5) # 10 Hz


class RRT:
    """
    Class for RRT planning
    """


    class Node:
        """
        RRT Node
        """


        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.path_x = []
            self.path_y = []
            self.parent = None

    class AreaBounds:

        def __init__(self, area):
            self.xmin = float(area[0])
            self.xmax = float(area[1])
            self.ymin = float(area[2])
            self.ymax = float(area[3])


    def __init__(self,
                 start,
                 goal,
                 obstacle_list,
                 rand_area,
                 expand_dis=3.0,
                 path_resolution=0.5,
                 goal_sample_rate=5,
                 max_iter=500,
                 play_area=None,
                 robot_radius=0.2,
                 ):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Random Sampling Area [min,max]
        play_area:stay inside this area [xmin,xmax,ymin,ymax]
        robot_radius: robot body modeled as circle with given radius

        """
        self.start = self.Node(start[0], start[1])
        self.end = self.Node(goal[0], goal[1])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        if play_area is not None:
            self.play_area = self.AreaBounds(play_area)
        else:
            self.play_area = None
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.node_list = []
        self.robot_radius = robot_radius

    def planning(self, animation=True):
        """
        rrt path planning

        animation: flag for animation on or off
        """

        self.node_list = [self.start]
        for i in range(self.max_iter):
            rnd_node = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]

            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)

            if self.check_if_outside_play_area(new_node, self.play_area) and \
               self.check_collision(
                   new_node, self.obstacle_list, self.robot_radius):
                self.node_list.append(new_node)

            if animation and i % 5 == 0:
                self.draw_graph(rnd_node)

            if self.calc_dist_to_goal(self.node_list[-1].x,
                                      self.node_list[-1].y) <= self.expand_dis:
                final_node = self.steer(self.node_list[-1], self.end,
                                        self.expand_dis)
                if self.check_collision(
                        final_node, self.obstacle_list, self.robot_radius):
                    return self.generate_final_course(len(self.node_list) - 1)

            if animation and i % 5:
                self.draw_graph(rnd_node)

        return None  # cannot find path

    def steer(self, from_node, to_node, extend_length=float("inf")):

        new_node = self.Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        if extend_length > d:
            extend_length = d

        n_expand = math.floor(extend_length / self.path_resolution)

        for _ in range(n_expand):
            new_node.x += self.path_resolution * math.cos(theta)
            new_node.y += self.path_resolution * math.sin(theta)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        d, _ = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)
            new_node.x = to_node.x
            new_node.y = to_node.y

        new_node.parent = from_node

        return new_node

    def generate_final_course(self, goal_ind):
        path = [[self.end.x, self.end.y]]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])

        return path

    def calc_dist_to_goal(self, x, y):
        dx = x - self.end.x
        dy = y - self.end.y
        return math.hypot(dx, dy)

    def get_random_node(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = self.Node(
                random.uniform(self.min_rand, self.max_rand),
                random.uniform(self.min_rand, self.max_rand))
        else:  # goal point sampling
            rnd = self.Node(self.end.x, self.end.y)
        return rnd

    def draw_graph(self, rnd=None):
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
            if self.robot_radius > 0.0:
                self.plot_circle(rnd.x, rnd.y, self.robot_radius, '-r')
        for node in self.node_list:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")

        for (ox, oy, size) in self.obstacle_list:
            self.plot_circle(ox, oy, size)

        if self.play_area is not None:
            plt.plot([self.play_area.xmin, self.play_area.xmax,
                      self.play_area.xmax, self.play_area.xmin,
                      self.play_area.xmin],
                     [self.play_area.ymin, self.play_area.ymin,
                      self.play_area.ymax, self.play_area.ymax,
                      self.play_area.ymin],
                     "-k")

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis("equal")
        plt.axis([-5, 20, -5, 20])
        plt.grid(True)
        plt.pause(0.01)

    @staticmethod
    def plot_circle(x, y, size, color="-b"):  # pragma: no cover
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, color)

    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        dlist = [(node.x - rnd_node.x)**2 + (node.y - rnd_node.y)**2
                 for node in node_list]
        minind = dlist.index(min(dlist))

        return minind

    @staticmethod
    def check_if_outside_play_area(node, play_area):

        if play_area is None:
            return True  # no play_area was defined, every pos should be ok

        if node.x < play_area.xmin or node.x > play_area.xmax or \
           node.y < play_area.ymin or node.y > play_area.ymax:
            return False  # outside - bad
        else:
            return True  # inside - ok

    @staticmethod
    def check_collision(node, obstacleList, robot_radius):

        if node is None:
            return False

        for (ox, oy, size) in obstacleList:
            dx_list = [ox - x for x in node.path_x]
            dy_list = [oy - y for y in node.path_y]
            d_list = [dx * dx + dy * dy for (dx, dy) in zip(dx_list, dy_list)]

            if min(d_list) <= (size+robot_radius)**2:
                return False  # collision

        return True  # safe

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta

def getCurvature(xRef, yRef):
    # Calculate gradient by the gradient of the X and Y vectors
    DX = np.gradient(xRef)
    D2X = np.gradient(DX)
    DY = np.gradient(yRef)
    D2Y = np.gradient(DY)
    curvature = (DX*D2Y - DY*D2X) / (DX**2 + DY**2)**(3/2)
    return curvature


def main(gx=15.0, gy=15):
    print("start " + __file__)

    # ====Search Path with RRT====
    obstacleList = [(5, 5, 1), (3, 6, 2), (3, 8, 2), (3, 10, 2), (7, 5, 2),
                    (9, 5, 2), (8, 10, 1)]  # [x, y, radius]
    """
    # Read the PGM file using OpenCV
    pgm_file = "my_map.pgm"
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
    obstacleList = []
    for i in range(map_height):
        for j in range(map_width):
            x = origin[0] + (j + 0.5) * resolution
            y = origin[1] + (map_height - i - 0.5) * resolution
            occupancy_map[i, j] = binary_img[i, j]
            if binary_img[i, j] == 0:
                obstacleList.append((x, y, 0.2)) # assuming robot radius is 0.8 meters

    # Save the occupancy grid map as a text file
    np.savetxt("occupancy_map.txt", occupancy_map, fmt="%d")

    # Visualize the occupancy grid map
    plt.imshow(occupancy_map, cmap="gray")
    plt.show()
"""




    # Set Initial parameters
    rrt = RRT(
        start=[0, 0],
        goal=[gx, gy],
        rand_area=[-5, 20],
        obstacle_list=obstacleList,
        # play_area=[0, 10, 0, 14]
        robot_radius=0.2
        )
    path = rrt.planning(animation=show_animation)

    if path is None:
        print("Cannot find path")
    else:
        print("found path!!")

        pathxy = np.array(path)
        xx = np.flip(pathxy[:,0])
        yy = np.flip(pathxy[:,1])
        if show_animation:
            rrt.draw_graph()
            plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
            plt.grid(True)
            plt.pause(0.01)  # Need for Mac
            plt.show()
##############################################################
        # Assuming you have an array of x and y coordinates from RRT
        x_rrt = xx
        y_rrt = yy

        # Create a cubic spline interpolation
        cs = CubicSpline(x_rrt, y_rrt)

        # Generate new x and y coordinates for the smoothed path
        x_smooth = np.linspace(x_rrt[0], x_rrt[-1], num=100)
        y_smooth = cs(x_smooth)

        print(x_smooth)
        # Plot the smoothed path
       
        plt.plot(x_rrt, y_rrt, 'o', label='RRT Path')
        plt.plot(x_smooth, y_smooth, label='Smoothed Path')
        plt.legend()
        plt.show()
###########################################################3
        x = x_smooth
        y = y_smooth

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
        plt.plot(xRef2s, yRef2, 'b')
        plt.xlabel('xRef2s')
        plt.ylabel('yRef2s')
        plt.show()

        xRef = xRef2s
        yRef = yRef2s
        thetaRef = np.zeros((len(gradbp), 1))
        for i in range(1, len(gradbp)):
            thetaRef[i, 0] = np.arctan2((yRef2[i]-yRef2[i-1]), (xRef2[i]-xRef2[i-1]))
            if (thetaRef[i, 0] < 0):
                thetaRef[i, 0] = np.pi + (np.pi + thetaRef[i, 0])
            else:
                pass
        thetaRefs = np.interp(gradbp, gradbp, thetaRef.flatten(), period=totalDistance)
        psi_o = thetaRefs[0]*(np.pi/180)

        plt.plot(gradbp, thetaRefs)
        plt.xlabel('distance')
        plt.ylabel('thetaRefs')
        plt.show()


        curvature = getCurvature(xRef2, yRef2)
        gradbp = np.linspace(0, distbp[-1], 1000)
        plt.plot(gradbp, curvature)
        plt.xlabel('distance')
        plt.ylabel('curvature')
        plt.show()

        path = np.column_stack((xRef2s, yRef2s, thetaRefs)).T
        file_path = os.path.expanduser("~/catkin_ws/path/path.csv")
        np.savetxt(file_path, path, delimiter=",")
        # Assuming path is a 2D NumPy array
        path_shape = path.shape
        print(path_shape)  # prints the shape of the path array

        # To get the number of rows or the number of waypoints, you can access the first dimension of the shape tuple
        num_waypoints = path_shape[0]
        print(num_waypoints)  # prints the number of waypoints in the path

        path_length = len(path)
        print(path_length)  # prints the length of the path list

##############################################################
        while not rospy.is_shutdown():
          
            # Create a new message using the custom message type
            msg = custom()
            """
            # Set the message fields
            msg.robot_name = "My Robot"
            msg.date_time = "2022-03-15"

            # Set the Pose2D message fields
            msg.location.x = 1.0
            msg.location.y = 2.0
            msg.location.theta = 3.0
            """
            # Set the integer arrays
            msg.x = xRef2s
            msg.y = yRef2s
            msg.theta = thetaRefs
            rospy.loginfo('ok---node is running')

            # Publish the message
            pub.publish(msg)

            # Sleep for the remainder of the loop period to maintain the loop rate
            rate.sleep()   

if __name__ == '__main__':
    try:
        main()
        
    except rospy.ROSInterruptException:
        pass



