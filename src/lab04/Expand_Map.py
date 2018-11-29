#!/usr/bin/env python
import sys
import rospy
import math
from src.lab03 import map_helper
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid, GridCells, Path, MapMetaData
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, PoseStamped, PoseArray, Pose



class Expand_Map:

    robot_radius = .1

    def __init__(self):
        """
        Use this node to expand the map to ensure that the turtlebot will not enter 
        a space too small for it to enter.
        """

        # Initialize node
        rospy.init_node("expand_map", log_level=rospy.DEBUG)

        # Subscribers
        rospy.Subscriber("map", OccupancyGrid, self.map_callback)

        # Publishers
        self.pub_expanded_grid = rospy.Publisher("local_costmap/expanded", GridCells, queue_size=10)
        # Full Path
        # Horizon Path

        # Service Calls
        self.dynamic_map = rospy.ServiceProxy('dynamic_map', GetMap)
        # Full Path
        # Horizon Path

        self.map = OccupancyGrid()

        self.rate = rospy.Rate(.5)

        while self.map is None and not rospy.is_shutdown():
            pass

    def euclidean_distance(self, point1, point2):
        """
            calculate the dist between two points
            :param point1: tuple of location
            :param point2: tuple of location
            :return: dist between two points
        """
        #Pythagorian theorem
        return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)
    pass

    def map_callback(self, msg):
        """
            This is a callback for the /map topic
            :param msg: map
            :return: None
        """
        expanded_map = self.expand(msg)
        occo_map = OccupancyGrid()
        occo_map.header = msg.header
        occo_map.data = expanded_map
        occo_map.info = msg.info
        self.expanded_map = occo_map
        self.map_pub.publish(occo_map)


    def handle_map(self, req):
        """
            Service call to get map and expand it
            :return:
        """



    def expand(self,my_map):
        #type: (OccupancyGrid) -> None
        """
            Expand the map and return it
            :param my_map: map
            :return: map
        """
        grid = GridCells()
        grid.header.frame_id = "/odom"
        grid.cell_height = my_map.info.resolution
        grid.cell_width = my_map.info.resolution
        grid.cells = []

        expanded_map = my_map
        occupied = 100

        #iterate through all
        cells = my_map.data
        for i, cell in enumerate(cells):

            # if occupied
            if cell > 0:
                point = map_helper.index1d_to_index2d(i, self.map)

                grid.cells.append(map_helper.map_to_world(point, self.map))

                neighbors = map_helper.get_neighbors(point, self.map)

                # iterate through neighbors
                for n in neighbors:
                    index_of_n = map_helper.index2d_to_index1d(n, self.map)
                    expanded_map.data[index_of_n] = occupied

                    grid.cells.append(map_helper.map_to_world(n, self.map))

                if self.map.info.resolution < .1:
                    # iterate through neighbors of neighbors
                    pass

            else:
                continue

        self.pub_expanded_grid.publish(grid)

        pass

    def get_map(self):
        self.map = self.dynamic_map()
        self.expand(self.map)

if __name__ == '__main__':
    expand = Expand_Map()
    rospy.loginfo("Expanding the Map")
