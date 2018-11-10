#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
import map_helper


class A_Star:

    def __init__(self):

        """
            This node handle A star paths requests.
            It is accessed using a service call. It can the publish grid cells
            to show the frontier,closed and path.
        """
        # Initialize node
        rospy.init_node("a_star", log_level=rospy.DEBUG)  # start node

        #Setup Map Publishers
        self.grid_pub = rospy.Publisher("local_costmap/obstacles", map_helper.GridCells, queue_size=10)

        # Setup Map Subscriber
        rospy.Subscriber("map", OccupancyGrid, self.dynamic_map_client)

        # Set map to none
        self.map = None
        rospy.logdebug("Initializing A_Star")

        self.rate = rospy.Rate(.5)


    def handle_a_star(self, req):

        """
            service call that uses A* to create a path.
            This can be altered to also grab the orentation for better heuristic
            :param req: GetPlan
            :return: Path()
        """
        pass

    def dynamic_map_client(self, new_map):

        """
            Service call to get map and set class variables
            This can be changed to call the expanded map
            :return:
        """
        rospy.logdebug("Getting map")

        self.map = new_map
        rospy.logdebug("Resolution is: %s" % new_map.info.resolution)

    def a_star(self, start, goal):
        """
            A*
            This is where the A* algorithum belongs
            :param start: tuple of start pose
            :param goal: tuple of goal pose
            :return: dict of tuples
        """
        pass

    def euclidean_heuristic(self, point1, point2):
        """
            calculate the dist between two points
            :param point1: tuple of location
            :param point2: tuple of location
            :return: dist between two points
        """

    pass

    def move_cost(self, current, next):
        """
              calculate the dist between two points
              :param current: tuple of location
              :param next: tuple of location
              :return: dist between two points
        """
        pass

    def reconstruct_path(self, start, goal, came_from):
        """
            Rebuild the path from a dictionary
            :param start: starting key
            :param goal: starting value
            :param came_from: dictionary of tuples
            :return: list of tuples
       """

    pass

    def optimize_path(self, path):
        """
            remove redundant points in hte path
            :param path: list of tuples
            :return: reduced list of tuples
        """
        pass

    def paint_cells(self, frontier, came_from):
        # type: (list, list) -> None
        """
            published cell of A* to Rviz
            :param frontier: tuples of the point on the frontier set
            :param came_from: tuples of the point on the closed set
            :return:
        """
        pass

    def publish_path(self, points):
        """
            Create a Path() and publishes the cells if Paint is true
            :param points: list of tuples of the path
            :return: Path()
        """
        pass

    def paint_grid_cells(self, Points = []):
        print "Painting"
        grid = map_helper.GridCells()
        grid.header.frame_id = "/odom"
        grid.cell_height = 1
        grid.cell_width = 1

        grid.cells = Points

        # Test values
        if not Points:
            for i in range(1, 10):
                point = map_helper.Point()
                point.x = i
                point.y = i

                grid.cells.append(point)


        print "Publishing"
        self.grid_pub.publish(grid)



if __name__ == '__main__':
    astar = A_Star()
    rospy.loginfo("Initializing A_Star")

    rospy.spin()
    pass
