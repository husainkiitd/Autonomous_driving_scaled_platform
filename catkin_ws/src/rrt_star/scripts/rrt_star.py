"""

Path planning Sample Code with RRT*

author: Atsushi Sakai(@Atsushi_twi)

"""

import math
import sys
import matplotlib.pyplot as plt
import pathlib
import cv2
import os
import numpy as np
from scipy.interpolate import interp1d
from rrt import RRT
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped

show_animation = False

start_pose = np.zeros(3)
goal_pose = np.zeros(3)
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
    start_pose[0] = pose_msg.pose.pose.position.x
    start_pose[1] = pose_msg.pose.pose.position.y
    start_pose[2] = angle(pose_msg.pose.pose.orientation.z)
    rospy.loginfo(start_pose)

def goal_pose_callback(pose_msg):
    global goal_pose 
    goal_pose[0] = pose_msg.pose.position.x
    goal_pose[1] = pose_msg.pose.position.y
    goal_pose[2] = angle(pose_msg.pose.orientation.z)
    rospy.loginfo(goal_pose)
    # ====Search Path with RRT====
    #_, _, obstacleList = get_circle_coords(30,30,5)
    ox, oy, obstacle_list = obstacle_from_map()    
    # [x,y,size(radius)]

    # Set Initial parameters
    rrt_star = RRTStar(
        start=[start_pose[0], start_pose[1]],
        goal=[goal_pose[0], goal_pose[1]],
        rand_area=[-2, 8],
        obstacle_list=obstacle_list,
        expand_dis=1,
        robot_radius=0.3)
    path = rrt_star.planning(animation=show_animation)

    if path is None:
        print("Cannot find path")
    else:
        print("found path!!")
        pathxy = np.array(path)
        xx = np.flip(pathxy[:,0])
        yy = np.flip(pathxy[:,1])
        path_Smoothing(xx,yy)
        # Draw final path
        rrt_star.draw_graph()
        plt.plot([x for (x, y) in path], [y for (x, y) in path], 'r--')
        plt.grid(True)
        plt.show()
        if show_animation:
            rrt_star.draw_graph()
            plt.plot([x for (x, y) in path], [y for (x, y) in path], 'r--')
            plt.grid(True)
            plt.show()

class RRTStar(RRT):
    """
    Class for RRT Star planning
    """

    class Node(RRT.Node):
        def __init__(self, x, y):
            super().__init__(x, y)
            self.cost = 0.0

    def __init__(self,
                 start,
                 goal,
                 obstacle_list,
                 rand_area,
                 expand_dis=0.10,
                 path_resolution=0.10,
                 goal_sample_rate=20,
                 max_iter=1000,
                 connect_circle_dist=50.0,
                 search_until_max_iter=False,
                 robot_radius=0.3):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Random Sampling Area [min,max]

        """
        super().__init__(start, goal, obstacle_list, rand_area, expand_dis,
                         path_resolution, goal_sample_rate, max_iter,
                         robot_radius=robot_radius)
        self.connect_circle_dist = connect_circle_dist
        self.goal_node = self.Node(goal[0], goal[1])
        self.search_until_max_iter = search_until_max_iter
        self.node_list = []

    def planning(self, animation=True):
        """
        rrt star path planning

        animation: flag for animation on or off .
        """

        self.node_list = [self.start]
        for i in range(self.max_iter):
            print("Iter:", i, ", number of nodes:", len(self.node_list))
            rnd = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd)
            new_node = self.steer(self.node_list[nearest_ind], rnd,
                                  self.expand_dis)
            near_node = self.node_list[nearest_ind]
            new_node.cost = near_node.cost + \
                math.hypot(new_node.x-near_node.x,
                           new_node.y-near_node.y)

            if self.check_collision(
                    new_node, self.obstacle_list, self.robot_radius):
                near_inds = self.find_near_nodes(new_node)
                node_with_updated_parent = self.choose_parent(
                    new_node, near_inds)
                if node_with_updated_parent:
                    self.rewire(node_with_updated_parent, near_inds)
                    self.node_list.append(node_with_updated_parent)
                else:
                    self.node_list.append(new_node)

            if animation:
                self.draw_graph(rnd)

            if ((not self.search_until_max_iter)
                    and new_node):  # if reaches goal
                last_index = self.search_best_goal_node()
                if last_index is not None:
                    return self.generate_final_course(last_index)

        print("reached max iteration")

        last_index = self.search_best_goal_node()
        if last_index is not None:
            return self.generate_final_course(last_index)

        return None

    def choose_parent(self, new_node, near_inds):
        """
        Computes the cheapest point to new_node contained in the list
        near_inds and set such a node as the parent of new_node.
            Arguments:
            --------
                new_node, Node
                    randomly generated node with a path from its neared point
                    There are not coalitions between this node and th tree.
                near_inds: list
                    Indices of indices of the nodes what are near to new_node

            Returns.
            ------
                Node, a copy of new_node
        """
        if not near_inds:
            return None

        # search nearest cost in near_inds
        costs = []
        for i in near_inds:
            near_node = self.node_list[i]
            t_node = self.steer(near_node, new_node)
            if t_node and self.check_collision(
                    t_node, self.obstacle_list, self.robot_radius):
                costs.append(self.calc_new_cost(near_node, new_node))
            else:
                costs.append(float("inf"))  # the cost of collision node
        min_cost = min(costs)

        if min_cost == float("inf"):
            print("There is no good path.(min_cost is inf)")
            return None

        min_ind = near_inds[costs.index(min_cost)]
        new_node = self.steer(self.node_list[min_ind], new_node)
        new_node.cost = min_cost

        return new_node

    def search_best_goal_node(self):
        dist_to_goal_list = [
            self.calc_dist_to_goal(n.x, n.y) for n in self.node_list
        ]
        goal_inds = [
            dist_to_goal_list.index(i) for i in dist_to_goal_list
            if i <= self.expand_dis
        ]

        safe_goal_inds = []
        for goal_ind in goal_inds:
            t_node = self.steer(self.node_list[goal_ind], self.goal_node)
            if self.check_collision(
                    t_node, self.obstacle_list, self.robot_radius):
                safe_goal_inds.append(goal_ind)

        if not safe_goal_inds:
            return None

        min_cost = min([self.node_list[i].cost for i in safe_goal_inds])
        for i in safe_goal_inds:
            if self.node_list[i].cost == min_cost:
                return i

        return None

    def find_near_nodes(self, new_node):
        """
        1) defines a ball centered on new_node
        2) Returns all nodes of the three that are inside this ball
            Arguments:
            ---------
                new_node: Node
                    new randomly generated node, without collisions between
                    its nearest node
            Returns:
            -------
                list
                    List with the indices of the nodes inside the ball of
                    radius r
        """
        nnode = len(self.node_list) + 1
        r = self.connect_circle_dist * math.sqrt(math.log(nnode) / nnode)
        # if expand_dist exists, search vertices in a range no more than
        # expand_dist
        if hasattr(self, 'expand_dis'):
            r = min(r, self.expand_dis)
        dist_list = [(node.x - new_node.x)**2 + (node.y - new_node.y)**2
                     for node in self.node_list]
        near_inds = [dist_list.index(i) for i in dist_list if i <= r**2]
        return near_inds

    def rewire(self, new_node, near_inds):
        """
            For each node in near_inds, this will check if it is cheaper to
            arrive to them from new_node.
            In such a case, this will re-assign the parent of the nodes in
            near_inds to new_node.
            Parameters:
            ----------
                new_node, Node
                    Node randomly added which can be joined to the tree

                near_inds, list of uints
                    A list of indices of the self.new_node which contains
                    nodes within a circle of a given radius.
            Remark: parent is designated in choose_parent.

        """
        for i in near_inds:
            near_node = self.node_list[i]
            edge_node = self.steer(new_node, near_node)
            if not edge_node:
                continue
            edge_node.cost = self.calc_new_cost(new_node, near_node)

            no_collision = self.check_collision(
                edge_node, self.obstacle_list, self.robot_radius)
            improved_cost = near_node.cost > edge_node.cost

            if no_collision and improved_cost:
                near_node.x = edge_node.x
                near_node.y = edge_node.y
                near_node.cost = edge_node.cost
                near_node.path_x = edge_node.path_x
                near_node.path_y = edge_node.path_y
                near_node.parent = edge_node.parent
                self.propagate_cost_to_leaves(new_node)

    def calc_new_cost(self, from_node, to_node):
        d, _ = self.calc_distance_and_angle(from_node, to_node)
        return from_node.cost + d

    def propagate_cost_to_leaves(self, parent_node):

        for node in self.node_list:
            if node.parent == parent_node:
                node.cost = self.calc_new_cost(parent_node, node)
                self.propagate_cost_to_leaves(node)


def obstacle_from_map():
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
    obstacle_list = []
    for i in range(map_height):
        for j in range(map_width):
            x = origin[0] + (j + 0.5) * resolution
            y = origin[1] + (map_height - i - 0.5) * resolution
            occupancy_map[i, j] = binary_img[i, j]
            if binary_img[i, j] == 0:
                ox.append(x)
                oy.append(y)
                obstacle_list.append((x,y,0.05))
                #obstacleList.append((x, y, 0.3)) # assuming robot radius is 0.8 meters

    # Save the occupancy grid map as a text file
    #np.savetxt("occupancy_map.txt", occupancy_map, fmt="%d")

    # Visualize the occupancy grid map
    plt.imshow(occupancy_map, cmap="gray")
    plt.show()
    print(len(ox))
    return ox, oy, obstacle_list

def get_circle_coords(center_x, center_y, radius):
    # create a list to store the x and y coordinates
    ox = []
    oy = []
    obstacle_list = []

    for i in range(60):
        ox.append(i)
        oy.append(0.0)
        obstacle_list.append((i, 0.0, 0.3))
    for i in range(60):
        ox.append(60.0)
        oy.append(i)
        obstacle_list.append((60.0, i, 0.3))
    for i in range(61):
        ox.append(i)
        oy.append(60.0)
        obstacle_list.append((i, 60.0, 0.3))
    for i in range(61):
        ox.append(0.0)
        oy.append(i)
        obstacle_list.append((0.0, i, 0.3))
    # loop through 360 degrees in steps of 10
    for angle_deg in range(0, 360, 1):
        # convert the angle to radians
        angle_rad = math.radians(angle_deg)
        
        # calculate the x and y coordinates of the point on the circle
        ox.append(center_x + radius * math.cos(angle_rad))
        oy.append(center_y + radius * math.sin(angle_rad))
        obstacle_list.append((center_x + radius * math.cos(angle_rad), center_y + radius * math.sin(angle_rad), 0.3))
        # add the coordinates to the list
        
    
    return ox, oy, obstacle_list

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


    plt.plot(xRef2s, yRef2, 'b')
    plt.xlabel('xRef2s')
    plt.ylabel('yRef2s')
    plt.show()


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


def main():
    print("start " + __file__)
    rospy.init_node('rrt_path_planner', anonymous=True)
    rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, start_pose_callback)
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, goal_pose_callback)
    rospy.spin()    
    print(__file__ + " done!!")  
 

if __name__ == '__main__':
    try:
        main()
        
    except rospy.ROSInterruptException:
        pass