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
import copy
from tf.transformations import euler_from_quaternion


class FrontierFinder:

    def __init__(self):
        """
            This node gets the expanded map through a service call, finds its frontier, and returns the map along with
            an array of points in frontiers sorted by distance to the robot.
        """
        # Initialize node
        rospy.init_node("frontier_finder", log_level=rospy.INFO)  # start node

        # Setup Map Publishers
        self.map_frontier_pub = rospy.Publisher("local_costmap/MapFrontier", GridCells, queue_size=10)
        self.map_frontier_group_pub = rospy.Publisher("local_costmap/MapFrontierGroups", GridCells, queue_size=10)
        self.pub_frontier_puffed = rospy.Publisher("local_costmap/FrontierExpanded", GridCells, queue_size=10)

        # Initialize variables
        self.position = None
        self.map = None

        # Setup service proxy
        self.get_map = rospy.ServiceProxy('get_expanded_map', GetMap)

        # Setup Map Subscriber
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        # Setup service server
        rospy.Service('get_frontiers', FrontierRequest, self.handle_get_frontier)

        self.listener = tf.TransformListener()

        self.max_radius = 2

    def handle_get_frontier(self, req):
        # type: (None) -> (OccupancyGrid, list)
        """
        handles the service call request for a frontier
        :param req: request
        :return: map, list of frontiers
        """
        # Get map
        occupancy_grid = self.get_map()
        self.map = occupancy_grid.map

        rospy.loginfo("Finding Frontier")
        start = rospy.get_time()

        # Find frontier points
        frontier_points = self.bfs_find_frontier_points()

        # Publish frontier points
        frontier_cells = map_helper.to_grid_cells(frontier_points.keys(), self.map, True)
        self.map_frontier_pub.publish(frontier_cells)


        # Group points
        frontiers = self.group_frontiers(frontier_points)

        # Display closest points
        frontier_group_cells = map_helper.to_grid_cells(frontiers, self.map, True)
        self.map_frontier_group_pub.publish(frontier_group_cells)

        frontier_poses = [map_helper.index2d_to_pose(cell, self.map) for cell in frontiers]

        rospy.loginfo("Finished frontier search %.2f" % (rospy.get_time() - start))

        return self.frontiered_map, frontier_poses

    def get_center(self, points):
        """
        Finds the point closest to the centroid
        :param points: list of tuples
        :return: point closest to centroid
        """
        # Find center
        sum_x, sum_y = 0, 0
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

        return self.cell_distance[closest], closest

    def get_closest(self, points):
        """
        Finds the point closest to the robot
        :param points: list of tuples
        :return: point closest to centroid
        """
        robot = self.position
        rospy.logdebug(robot)

        # Guess closest
        shortest_distance = self.cell_distance[points[0]]
        closest = points[0]

        # Find closest
        for point in points:
            dist = self.cell_distance[point]
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
        # position = msg.pose.pose.position
        # new_pose = PoseStamped()
        # new_pose.pose.position = position
        # coords = (position.x, position.y)
        # if self.map and self.map.info.resolution:
        #     self.position = map_helper.world_to_index2d(coords, self.map)
        #
        # self.world_position = position

        new_pose = PoseStamped()

        (trans, rot) = self.listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
        new_pose.pose.position.x = trans[0]
        new_pose.pose.position.y = trans[1]
        position = new_pose.pose.position

        coords = (position.x, position.y)
        if self.map and self.map.info.resolution:
            self.position = map_helper.world_to_index2d(coords, self.map)

        self.world_position = position

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

    def bfs_find_frontier_points(self):
        """
        iterates through the map to find points along the frontier using bfs
        :return: dictionary where keys are the point tuples and values are one element arrays of the same frontier point
        """
        frontier = {}
        self.cell_distance = {}
        self.puffed_dict = {}

        self.frontiers_to_paint = set()

        self.frontiered_map = copy.copy(self.map)
        self.new_occupancy = list(self.frontiered_map.data)
        count = 0
        start = map_helper.world_to_index2d((self.world_position.x, self.world_position.y), self.map)

        visited, queue = set(), [start]
        while queue:
            vertex = queue.pop(0)
            index1d = map_helper.index2d_to_index1d(vertex, self.map)

            # if unoccupied
            if 0 <= self.map.data[index1d] < 100:
                if map_helper.get_neighbors(vertex, self.map, -1):  # Checks if there are unknown neighbors
                    frontier[vertex] = [vertex]
                    self.puff_point(vertex)
                    self.cell_distance[vertex] = count
                    count += 1

            if vertex not in visited:
                visited.add(vertex)
                neighbors = map_helper.get_neighbors_8count(vertex, self.map)

                if neighbors is None:
                    neighbors = set()
                else:
                    neighbors = set(neighbors)

                queue.extend(neighbors - visited)

        grid = map_helper.to_grid_cells(list(self.frontiers_to_paint), self.map)
        self.pub_frontier_puffed.publish(grid)


        self.frontiered_map.data = tuple(self.new_occupancy)
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
            heapq.heappush(heap, self.get_center(group))
        return [heapq.heappop(heap)[1] for i in range(len(heap))]

    def puff_point(self, point):
        """
        Looks at all neighbors in radius of a point and, if unoccupied, makes them occupied
        :param point: tuple of index2d
        :return: adds to globals cells_to_paint and new_occupancy
        """
        steps_out = 0

        visit = map_helper.get_neighbors(point, self.map)

        self.puffed_dict[point] = steps_out

        while steps_out < self.max_radius and not rospy.is_shutdown():
            neighbors_to_expand = visit
            visit = []

            steps_out += 1

            for n in neighbors_to_expand:

                if n in self.puffed_dict and self.puffed_dict[n] < steps_out:
                    continue
                else:
                    self.puffed_dict[n] = steps_out

                    index_of_n = map_helper.index2d_to_index1d(n, self.map)

                    curr_value = self.new_occupancy[index_of_n]
                    function_value = self.antigrav_expansion(steps_out)

                    # graph takes in max between the original value and the newly calculated value
                    # only really affects cases of overlap
                    new_value = max(curr_value, function_value)
                    self.new_occupancy[index_of_n] = new_value

                    worldpt = map_helper.index2d_to_world(n, self.map)

                    if worldpt not in self.frontiers_to_paint:
                        self.frontiers_to_paint.add(worldpt)

                    visit = visit + map_helper.get_neighbors(n, self.map)

    def antigrav_expansion(self, step):
        """
        function applied to steps out to determine the value of the occupany grid

        :param step:
        :return:
        """
        max = self.max_radius + 1

        if step < max:
            # linear function to deter robot from walls
            # return int(-100/(max - min) * (step - min) + 100)

            # exponential
            val = -100.0/(max)**2 * (step)**2 + 100

            if val == 100:
                val = 90

            return val
        else:
            # free space
            return 0


if __name__ == '__main__':
    finder = FrontierFinder()
    # rate = rospy.Rate(3000)
    # while not rospy.is_shutdown():
    #     finder.handle_get_frontier(None)
    #     rate.sleep()
    rospy.spin()
