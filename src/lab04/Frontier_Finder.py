#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid, GridCells, Path, Odometry
from nav_msgs.srv import GetMap
from geometry_msgs.msg import PointStamped, Pose, PoseStamped, Point
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'lab03'))
import map_helper
import heapq
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
        self.map_frontier_group_pub = rospy.Publisher("local_costmap/MapFrontierGroups", GridCells, queue_size=10)

        # Initialize variables
        self.position = None
        self.map = None

        # Setup service proxy
        self.get_map = rospy.ServiceProxy('get_expanded_map', GetMap)

        # Setup Map Subscriber
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        # Setup service server
        rospy.Service('get_frontiers', FrontierRequest, self.handle_get_frontier)

    def handle_get_frontier(self, req):
        # type: (None) -> (OccupancyGrid, PoseStamped)
        """
        handles the service call request for a frontier
        :param req: request
        :return: map, list of frontiers
        """
        # Get map
        occupancy_grid = self.get_map()
        self.map = occupancy_grid.map
        rospy.logdebug(self.map.info)

        # Find frontier points
        frontier_points = self.find_frontier_points()

        # Publish frontier points
        frontier_cells = map_helper.to_grid_cells(frontier_points.keys(), self.map, True)
        self.map_frontier_pub.publish(frontier_cells)

        # Group points
        frontiers = self.group_frontiers(frontier_points)

        # Display closest points
        frontier_group_cells = map_helper.to_grid_cells(frontiers, self.map, True)
        self.map_frontier_group_pub.publish(frontier_group_cells)
        rospy.logdebug(frontiers)

        frontier_poses = [map_helper.index2d_to_pose(cell, self.map) for cell in frontiers]
        return occupancy_grid, frontier_poses[0]

    def get_closest(self, points):
        """
        Finds the point closest to the robot
        :param points: list of tuples
        :return: point closest to centroid
        """
        robot = self.position

        # Guess closest
        shortest_distance = map_helper.euclidean_distance(points[0], robot)
        closest = points[0]

        # Find closest
        for point in points:
            dist = map_helper.euclidean_distance(point, robot)
            if dist < shortest_distance:
                shortest_distance = dist
                closest = point
        return shortest_distance, closest

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
        coords = (position.x, position.y)
        if self.map:
            self.position = map_helper.world_to_index2d(coords, self.map)

    def find_frontier_points(self):
        """
        iterates through the map to find points along the frontier
        :return: dictionary where keys are the point tuples and values are one element arrays of the same frontier point
        """
        frontier = {}

        cells = self.map.data
        for i, cell in enumerate(cells):
            # if occupied
            if cell == 0:
                point = map_helper.index1d_to_index2d(i, self.map)
                if map_helper.get_neighbors(point, self.map, -1):  # Checks if there are unknown neighbors
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
        if cell in frontier_cells:
            del frontier_cells[cell]

        # Recursively check neighbors
        for neighbor in self.get_neighborhood(frontier_cells, cell):
            self.group_cells(frontier_cells, group, neighbor)

    def group_frontiers(self, frontier_cells):
        """
        Merges nearby points into single frontiers
        :param frontier_cells: dictionary of cells
        :return: sorted list of the closest point in each frontier
        """
        heap = []
        while len(frontier_cells):
            cell, group = frontier_cells.popitem()
            self.group_cells(frontier_cells, group, cell)
            heapq.heappush(heap, self.get_closest(group))
        return [heapq.heappop(heap)[1] for i in range(len(heap))]


if __name__ == '__main__':
    finder = FrontierFinder()
    rate = rospy.Rate(3000)
    while not rospy.is_shutdown():
        finder.handle_get_frontier(None)
        rate.sleep()
    rospy.spin()
