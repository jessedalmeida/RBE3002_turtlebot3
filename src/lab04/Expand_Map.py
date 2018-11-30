#!/usr/bin/env python
import sys
sys.path.insert(0, '/home/jfdalmeida/catkin_ws/src/RBE3002Code19/src/lab03')
import rospy
import math
import map_helper
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
        # expanded_map = self.expand(msg)
        # occo_map = OccupancyGrid()
        # occo_map.header = msg.header
        # occo_map.data = expanded_map
        # occo_map.info = msg.info
        # self.expanded_map = occo_map
        # self.map_pub.publish(occo_map)


    def handle_map(self, req):
        """
            Service call to get map and expand it
            :return:
        """



    def expand(self, my_map):
        #type: (OccupancyGrid) -> None
        """
            Expand the map and return it
            :param my_map: map
            :return: map
        """
        rospy.logdebug("Expanding the map")

        grid = GridCells()
        grid.header.frame_id = "/odom"
        grid.cell_height = my_map.info.resolution
        grid.cell_width = my_map.info.resolution
        cells_to_paint = []

        expanded_map = my_map
        occupied = 100

        #iterate through all
        cells = my_map.data
        for i in range(len(cells)):
            occupation = cells[i]
            # if occupied
            if occupation > 0:
                point = map_helper.index1d_to_index2d(i, self.map)

                # rospy.logdebug("Index %s Occupation %s Point %s" %(i, occupation, point))
                cells_to_paint.append(map_helper.map_to_world(point, self.map))

                neighbors = map_helper.get_neighbors(point, self.map)

                # iterate through neighbors
                for n in neighbors:
                    index_of_n = map_helper.index2d_to_index1d(n, self.map)
                    expanded_map.data[index_of_n]

                    cells_to_paint.append(map_helper.map_to_world(n, self.map))

                if self.map.info.resolution < .1:
                    # iterate through neighbors of neighbors
                    pass

            else:
                continue

        for c in cells_to_paint:
            world = map_helper.index2d_to_world(c, self.map)

            p = Point()
            p.x = world[0]
            p.y = world[1]

            grid.cells.append(p)


        rospy.logdebug("Publishing Grid Cells")
        self.pub_expanded_grid.publish(grid)

        pass

    def get_map(self):
        self.map = self.dynamic_map()
        self.expand(self.map)

if __name__ == '__main__':
    expanded = Expand_Map()
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        expanded.expand(expanded.map)
        rate.sleep()

    rospy.spin()


    rospy.loginfo("Expanding the Map")
