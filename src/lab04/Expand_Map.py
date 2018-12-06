#!/usr/bin/env python
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'lab03'))
import rospy
import math
import map_helper
import copy
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid, GridCells, Path, MapMetaData
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

        # Publishers
        self.pub_expanded_grid = rospy.Publisher("local_costmap/expanded", GridCells, queue_size=10)
        # Full Path
        # Horizon Path

        # Service Calls
        # self.dynamic_map = rospy.ServiceProxy('dynamic_map', GetMap)
        # Full Path
        # Horizon Path

        # Setup service server
        rospy.Service('get_expanded_map', GetMap, self.handle_map)

        self.map = OccupancyGrid()

        self.rate = rospy.Rate(.5)

        # Conts
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

        rospy.logdebug("Getting map")
        self.map = msg

        # Show map details for debugging
        rospy.logdebug("Resolution is: %s" % msg.info.resolution)
        x_index_offset = self.map.info.origin.position.x
        y_index_offset = self.map.info.origin.position.y
        rospy.logdebug("Map Origin: x: %s y: %s" % (x_index_offset, y_index_offset))

        width = self.map.info.width
        height = self.map.info.height
        rospy.logdebug("Map Width: %s Height: %s" % (width, height))

    def handle_map(self, req):
        """
            Service call to get map and expand it
            :return:
        """
        rospy.loginfo("Expanding Map")
        self.get_map()
        return self.expanded_map

    def expand(self, my_map):
        #type: (OccupancyGrid) -> None
        """
            Expand the map and return it
            :param my_map: map
            :return: map
        """

        self.expanded_map = copy.copy(my_map)
        self.new_occupancy = list(self.expanded_map.data)

        self.cells_to_paint = []

        if self.first_run:
            self.already_expanded = [False] * len(self.new_occupancy)

        # iterate through all
        cells = my_map.data
        for i in range(len(cells)):
            # paint around radius of a point of wall
            if self.already_expanded[i]:
                self.new_occupancy[i] == 100

            if cells[i] == 100 and not self.already_expanded[i]:
            # if cells[i] == 100:
                self.already_expanded[i] = True
                point = map_helper.index1d_to_index2d(i, self.map)
                self.puff_point(point)

        grid = map_helper.to_grid_cells(self.remove_duplicates(self.cells_to_paint), self.map)
        self.pub_expanded_grid.publish(grid)

        self.expanded_map.data = tuple(self.new_occupancy)
        rospy.logdebug("Expansion complete")

    def puff_point(self, point):
        """
        Looks at all neighbors in radius of a point and, if unoccupied, makes them occupied
        :param point: tuple of index2d
        :return:
        """
        # rospy.logdebug("Puffing point")

        radius = 2 * int(math.ceil(self.robot_radius / self.map.info.resolution))
        steps_out = 0

        visit = map_helper.get_neighbors(point, self.map)

        while steps_out < radius and not rospy.is_shutdown():
            # rospy.logdebug("Current r step: %s Visited: %s" % (steps_out, visit))
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
        # self.map = self.dynamic_map()
        self.expand(self.map)
        pass


if __name__ == '__main__':
    expanded = Expand_Map()
    # rate = rospy.Rate(1)

    # while not rospy.is_shutdown():
    #     expanded.expand(expanded.map)
    #     rate.sleep()

    rospy.spin()


    rospy.loginfo("Expanding the Map")
