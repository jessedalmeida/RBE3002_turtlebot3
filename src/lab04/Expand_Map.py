#!/usr/bin/env python
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'lab03'))
import rospy
import math
import map_helper
import copy
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid, GridCells, Odometry, Path, MapMetaData
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, PoseStamped, PoseArray, Pose
import cProfile

class Expand_Map:

    def __init__(self):
        """
        Use this node to expand the map to ensure that the turtlebot will not enter 
        a space too small for it to enter.
        """

        # Initialize node
        rospy.init_node("expand_map", log_level=rospy.DEBUG)

        # Subscribers
        rospy.Subscriber("map", OccupancyGrid, self.map_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        # Publishers
        self.pub_expanded_grid = rospy.Publisher("local_costmap/expanded", GridCells, queue_size=10)
        self.pub_soft_expanded = rospy.Publisher("local_costmap/lessExpanded", GridCells, queue_size=10)

        # Setup service server
        rospy.Service('get_expanded_map', GetMap, self.handle_map)

        # Consts
        self.map = OccupancyGrid()
        self.rate = rospy.Rate(.5)
        self.robot_radius = .1
        self.first_run = True

        while self.map is None and not rospy.is_shutdown():
            pass

    def map_callback(self, msg):
        # type: (OccupancyGrid) -> None
        """
            This is a callback for the /map topic
            :param msg: map
            :return: None
        """
        self.map = msg

    def odom_callback(self, msg):
        # type: (Odometry) -> None
        """
        update the state of the robot
        :type msg: Odom
        :return:
        """
        self.curr_position = msg.pose.pose.position

    def handle_map(self, req):
        """
            Service call to get map and expand it
            :return:
        """
        rospy.loginfo("Expanding Map")
        now = rospy.get_time()
        self.get_map()

        diff = rospy.get_time() - now
        rospy.loginfo("Expansion complete: %s" % diff)
        return self.expanded_map

    def expand(self):
        """
            Expand the map and return it
            :param my_map: map
            :return: map
        """
        rospy.logdebug("Expanding the map")

        rospy.logdebug("Start Node: %s" % self.curr_position)
        start_time = rospy.get_time()

        self.walls_to_paint = set()
        self.softwalls_to_paint = set()

        self.expanded_map = copy.copy(self.map)
        self.new_occupancy = list(self.expanded_map.data)
        self.min_radius = int(math.ceil(self.robot_radius / self.map.info.resolution))
        self.max_radius = int(2 * math.ceil(self.robot_radius / self.map.info.resolution))

        rospy.logdebug("Min: %s Max: %s" % (self.min_radius, self.max_radius))

        useTime = 0

        now = rospy.get_time()

        # iterate through all
        cells = self.map.data
        len_cells = len(cells)

        rospy.logdebug("Length %s" % len_cells)
        for i in range(len(cells)):

            rospy.logdebug_throttle(2, "%s of cells searched" % (i))
            # paint around radius of a point of wall
            if cells[i] == 100:
                startPuff = rospy.get_time()
                point = map_helper.index1d_to_index2d(i, self.map)
                self.puff_point(point)
                useTime += rospy.get_time() - startPuff

            # if rospy.get_time() - now > 2:
            #     now = rospy.get_time()
            #     self.update_grids()

        iterateTime = rospy.get_time() - start_time
        rospy.logdebug("Iteration Time: %s" % (iterateTime))
        rospy.logdebug("Puff Time: %s  Percentage used: %s" % (useTime, useTime / iterateTime))

        rospy.logdebug("Publishing")
        self.update_grids()

        self.expanded_map.data = tuple(self.new_occupancy)

    def bfs_expand(self):
        """
        Start at current position and use bfs to only search the immediate map area
        :return:
        """
        rospy.logdebug("Starting BFS")

        start = map_helper.world_to_index2d((self.curr_position.x, self.curr_position.y), self.map)

        rospy.logdebug("Start Node: %s" % self.curr_position)
        start_time = rospy.get_time()

        self.puffed_dict = {}

        self.walls_to_paint = set()
        self.softwalls_to_paint = set()

        self.expanded_map = copy.copy(self.map)
        self.new_occupancy = list(self.expanded_map.data)
        self.min_radius = int(math.ceil(self.robot_radius / self.map.info.resolution))
        self.max_radius = int(2 * math.ceil(self.robot_radius / self.map.info.resolution))

        useTime = 0

        rospy.logdebug("Min: %s Max: %s" % (self.min_radius, self.max_radius))

        # used = 0
        # cells_len = len(self.new_occupancy)
        # for i in range(cells_len):
        #     val = self.new_occupancy[i]
        #
        #     if val != -1:
        #         used += 1
        #
        # rospy.logdebug("Total Size: %s Usable Size: %s" %(cells_len, used))

        cells_searched = 0

        # usable_map_size = 25000 * (.05/self.map.info.resolution)**2

        now = rospy.get_time()

        visited, queue = set(), [start]
        while queue:
            vertex = queue.pop(0)

            cells_searched += 1

            # rospy.logdebug_throttle(2, "Cells searched so far %s" % cells_searched)
            # rospy.loginfo_throttle(1, "%.2f %% of the way through" % (cells_searched/used * 100))
            index1d = map_helper.index2d_to_index1d(vertex, self.map)

            if self.map.data[index1d] == 100:
                startPuff = rospy.get_time()
                self.puff_point(vertex)
                useTime += rospy.get_time() - startPuff

            if vertex not in visited:
                visited.add(vertex)
                neighbors = map_helper.get_neighbors_bfs(vertex, self.map)

                if neighbors is None:
                    neighbors = set()
                else:
                    neighbors = set(neighbors)

                queue.extend(neighbors - visited)

        iterateTime = rospy.get_time() - start_time
        rospy.logdebug("Iteration Time: %s" % (iterateTime))
        rospy.logdebug("Puff Time: %s  Percentage used: %s" % (useTime, useTime/iterateTime))


        rospy.logdebug("Publishing")
        self.update_grids()

        self.expanded_map.data = tuple(self.new_occupancy)

    def puff_point(self, point):
        """
        Looks at all neighbors in radius of a point and, if unoccupied, makes them occupied
        :param point: tuple of index2d
        :return: adds to globals cells_to_paint and new_occupancy
        """
        steps_out = 0

        visit = map_helper.get_neighbors(point, self.map)

        self.puffed_dict[point] = steps_out

        x = 1

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

                    if new_value == 100:
                        self.walls_to_paint.add(worldpt)
                    elif 0 < new_value and new_value < 100:
                        self.softwalls_to_paint.add(worldpt)

                    visit = visit + map_helper.get_neighbors(n, self.map)

    def antigrav_expansion(self, step):
        """
        function applied to steps out to determine the value of the occupany grid

        :param step:
        :return:
        """
        min = self.min_radius
        max = self.max_radius + 1

        if step <= min:
            # returns wall so that robot absolutely does not go in here
            return 100
        elif step > min and step < max:
            # linear function to deter robot from walls
            return int(-100/(max - min) * (step - min) + 100)
        else:
            # free space
            return 0

    def update_grids(self):
        hard_grid = map_helper.to_grid_cells(list(self.walls_to_paint), self.map)
        self.pub_expanded_grid.publish(hard_grid)

        soft_grid = map_helper.to_grid_cells(list(self.softwalls_to_paint), self.map)
        self.pub_soft_expanded.publish(soft_grid)

        return ""

    def get_map(self):
        # cProfile.runctx("self.bfs_expand()", globals(), locals())
        self.bfs_expand()
        # self.expand()
        pass


if __name__ == '__main__':
    expanded = Expand_Map()
    rospy.spin()
    rospy.loginfo("Expanding the Map")
