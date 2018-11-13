#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PointStamped, Pose
import map_helper
from PriorityQueue import PriorityQueue


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
        self.obstacles_pub = rospy.Publisher("local_costmap/obstacles", map_helper.GridCells, queue_size=10)

        # Setup Map Subscriber
        rospy.Subscriber("map", OccupancyGrid, self.dynamic_map_client)
        rospy.Subscriber("/clicked_point", PointStamped, self.astar_goal_client)

        # Set map to none
        self.map = None
        rospy.logdebug("Initializing A_Star")


        self.goal = PointStamped()
        self.pose = Pose()

        self.rate = rospy.Rate(.5)

        while self.map is None and not rospy.is_shutdown():
            pass


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
        x_index_offset = self.map.info.origin.position.x
        y_index_offset = self.map.info.origin.position.y
        rospy.logdebug("Map Origin: x: %s y: %s" % (x_index_offset, y_index_offset))

    def astar_goal_client(self, point):
        """
        Subscriber client to get the published goal point
        :param point: goal
        """
        rospy.logdebug("New goal: %s %s" % (point.point.x, point.point.y))
        self.goal = point

    def a_star(self, start, goal):
        """
            A*
            This is where the A* algorithum belongs
            :param start: tuple of start pose
            :param goal: tuple of goal pose
            :return: dict of tuples
        """
        start = map_helper.convert_location(start, self.map)
        goal = map_helper.convert_location(goal, self.map)

        frontier = PriorityQueue()
        frontier.put(start, 0)
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0

        while not frontier.empty():
            current = frontier.get()
            rospy.logdebug("Current Node %s " % (current, ))

            if current == goal:
                break

            # for next in graph.neighbors(current):
            for next in map_helper.get_neighbors(current, self.map):
                rospy.logdebug("Next node %s" % (next,))

                new_cost = cost_so_far[current] + self.move_cost(current, next)
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost
                    frontier.put(next, priority)
                    came_from[next] = current
        # TODO get frontier to list
        rospy.logdebug(frontier)

    def tester(self, point):
        a = map_helper.convert_location(point, self.map)
        rospy.logdebug("Neighbors of %s are %s " % (point, a))

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
        return 0

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

    def paint_grid_cells(self):
        """publishes grid cells with the given coordinates"""
        while self.map is None and not rospy.is_shutdown():
            pass

        # obstacles = [(i, i) for i in range(10)]
        if self.goal:
            obstacles = [(self.goal.point.x, self.goal.point.y)]
        else:
            obstacles = [(0,0)]


        cells = map_helper.to_grid_cells(obstacles, self.map)
        # rospy.logdebug(cells.cells)
        self.obstacles_pub.publish(cells)


if __name__ == '__main__':
    astar = A_Star()
    rospy.loginfo("Initializing A_Star")


    rate = rospy.Rate(1)
    # while not rospy.is_shutdown():
        # astar.paint_grid_cells()
    astar.a_star((0, 0), (3, 3))
        # rate.sleep()
    # rospy.spin()
    pass
