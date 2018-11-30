#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid, GridCells, Path, Odometry
from nav_msgs.srv import GetMap
from geometry_msgs.msg import PointStamped, Pose, PoseStamped, Point
import map_helper
import sys
sys.path.insert(map_helper)
from PriorityQueue import PriorityQueue
import math
from rbe3002.srv import FrontierRequest
import tf
from tf.transformations import euler_from_quaternion


class FrontierFinder:

    def __init__(self):
        """
            This node gets the expanded map through a service call, finds its frontier, and returns the map along with
            an array of points in frontiers sorted by distance to the robot.
        """
        # Initialize node
        rospy.init_node("frontier_finder", log_level=rospy.DEBUG)  # start node

        # Setup Map Publishers
        self.map_frontier_pub = rospy.Publisher("local_costmap/MapFrontier", GridCells, queue_size=10)

        # Initialize variables
        self.pose = None
        self.map = None

        # Setup service proxy
        self.get_map = rospy.ServiceProxy('get_expanded_map', GetMap)

        # Setup Map Subscriber
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        # Setup service server
        rospy.Service('get_frontiers', FrontierRequest, self.handle_get_frontier)

    def handle_get_frontier(self, req):
        # type: (None) -> (OccupancyGrid, list)
        """
        handles the service call request for a frontier
        :param req: request
        :return: map, list of frontiers
        """
        self.map = self.get_map()
        rospy.log(req)

    def get_center(self, points):
        """
        Finds the point closest to the centroid
        :param points: list of tuples
        :return: point closest to centroid
        """
        # Find center
        sum_x, sum_y = 0
        for point in points:
            sum_x += point[0]
            sum_y += point[1]
        center = (sum_x / len(points), sum_y / len(points))

        # Guess closest
        shortest_distance = map_helper.euclidean_distance(points[0], center)
        closest = points[0]

        # Find closest
        for point in points:
            dist = map_helper.euclidean_distance(point, center)
            if dist < shortest_distance:
                shortest_distance = dist
                closest = point
        return closest

    def odom_callback(self, msg):
        # type: (Odometry) -> None
        """
        update the state of the robot
        :type msg: Odom
        :return:
        """
        position = msg.pose.pose.position
        new_pose = PoseStamped()
        new_pose.pose.position = position
        self.pose = new_pose

    def find_frontier_points(self):
        """
        iterates through the map to find points along the frontier
        :return: dictionary where keys are the point tuples and values are one element arrays of the same frontier point
        """
        frontier = {}

        cells = self.map
        for i, cell in enumerate(cells):
            # if occupied
            if cell == 0:
                point = map_helper.index1d_to_index2d(i, self.map)
                if map_helper.get_neighbors(point, cells, -1):  # Checks if there are unknown neighbors
                    frontier[point] = [point]

        return frontier

    def get_neighborhood(self, frontier_cells, cell):
        """
        Returns a list of the frontier cells within 8-distance of cell
        :param frontier_cells: dictionary of cells
        :param cell: cell to search around
        :return: list of cells nearby
        """
        cell_x, cell_y = cell
        radius = range(-1, 2)

        neighborhood = []
        for x in radius:
            for y in radius:
                candidate = (cell_x + x, cell_y + y)
                if candidate in frontier_cells:
                    neighborhood += [candidate]
        return neighborhood

    def group_cells(self, frontier_cells, group, cell):
        """
        Recursively pulls in all nearby cells in the frontier into the group
        :param frontier_cells: dictionary of ungrouped cells in the frontier
        :param group: Group of cell to merge neighbors into
        :param cell: Cell to search around
        """
        # Add to group
        group += [cell]

        # Remove from frontier of ungrouped cells
        del frontier_cells[cell]

        # Recursively check neighbors
        for neighbor in self.get_neighborhood(frontier_cells, cell):
            self.group_cells(frontier_cells, group, neighbor)

    def group_frontiers(self, frontier_cells):
        """
        Merges nearby points into single frontiers
        :param frontier_cells: dictionary of cells
        :return: dictionary of combined frontiers
        """
        frontiers = {}
        while len(frontier_cells):
            cell, group = frontier_cells.popitem()
            self.group_cells(frontier_cells, group, cell)
            frontiers[self.get_center(group)] = group
        return frontiers

