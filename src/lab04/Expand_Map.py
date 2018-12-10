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


class Expand_Map:

    def __init__(self):
        """
        Use this node to expand the map to ensure that the turtlebot will not enter 
        a space too small for it to enter.
        """

        # Initialize node
        rospy.init_node("expand_map", log_level=rospy.INFO)

        # Subscribers
        rospy.Subscriber("map", OccupancyGrid, self.map_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        # Publishers
        self.pub_expanded_grid = rospy.Publisher("local_costmap/expanded", GridCells, queue_size=10)

        # Setup service server
        rospy.Service('get_expanded_map', GetMap, self.handle_map)

        # Consts
        self.map = OccupancyGrid()
        self.rate = rospy.Rate(.5)
        self.robot_radius = .1
        self.first_run = True
        self.cells_to_paint = []

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

    def bfs_expand(self):
        """
        Start at current position and use bfs to only search the immediate map area
        :return:
        """
        rospy.logdebug("Starting BFS")

        start = map_helper.world_to_index2d((self.curr_position.x, self.curr_position.y), self.map)

        rospy.logdebug("Start Node: %s" % self.curr_position)
        now = rospy.get_time()

        self.expanded_map = copy.copy(self.map)
        self.new_occupancy = list(self.expanded_map.data)
        self.radius = int(1.4 * math.ceil(self.robot_radius / self.map.info.resolution))
        useTime = 0

        visited, queue = set(), [start]
        while queue:
            vertex = queue.pop(0)
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


        iterateTime = rospy.get_time() - now
        rospy.logdebug("Iteration Time: %s" % (iterateTime))
        rospy.logdebug("Puff Time: %s  Percentage used: %s" % (useTime, useTime/iterateTime))


        rospy.logdebug("Publishing")
        grid = map_helper.to_grid_cells(self.cells_to_paint, self.map)
        self.pub_expanded_grid.publish(grid)

        self.expanded_map.data = tuple(self.new_occupancy)

    def puff_point(self, point):
        """
        Looks at all neighbors in radius of a point and, if unoccupied, makes them occupied
        :param point: tuple of index2d
        :return: adds to globals cells_to_paint and new_occupancy
        """
        steps_out = 0

        visit = map_helper.get_neighbors(point, self.map)

        while steps_out < self.radius and not rospy.is_shutdown():
            neighbors_to_expand = visit
            visit = []

            steps_out += 1

            for n in neighbors_to_expand:
                index_of_n = map_helper.index2d_to_index1d(n, self.map)

                self.new_occupancy[index_of_n] = 100
                worldpt = map_helper.index2d_to_world(n, self.map)
                self.cells_to_paint.append(worldpt)
                visit = visit + map_helper.get_neighbors(n, self.map)

    def remove_duplicates(self, list):
        final_list = []
        for point in list:
            if point not in final_list:
                final_list.append(point)
        return final_list

    def get_map(self):
        self.bfs_expand()
        pass


if __name__ == '__main__':
    expanded = Expand_Map()
    rospy.spin()


    rospy.loginfo("Expanding the Map")
