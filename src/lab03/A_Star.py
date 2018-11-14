#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid, GridCells
from geometry_msgs.msg import PointStamped, Pose, PoseStamped
import map_helper
from PriorityQueue import PriorityQueue
import math
from rbe3002.srv import *
import tf
from tf.transformations import euler_from_quaternion


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
        self.obstacles_pub = rospy.Publisher("local_costmap/obstacles", GridCells, queue_size=10)
        self.path_pub = rospy.Publisher("local_costmap/path", GridCells, queue_size=10)
        self.point_pub = rospy.Publisher("/point_cell", GridCells, queue_size=10)
        self.wavefront_pub = rospy.Publisher("local_costmap/wavefront", GridCells, queue_size=10)

        # Setup Map Subscriber
        rospy.Subscriber("map", OccupancyGrid, self.dynamic_map_client)

        service = rospy.Service('make_path', MakePath, self.handle_a_star)

        # Set map to none
        self.map = None
        rospy.logdebug("Initializing A_Star")


        self.goal = PoseStamped()
        self.start = PoseStamped()
        self.pose = Pose()

        self.rate = rospy.Rate(.5)

        while self.map is None and not rospy.is_shutdown():
            pass


    def handle_a_star(self, req):
        # type: (MakePath) -> None

        """
            service call that uses A* to create a path.
            This can be altered to also grab the orentation for better heuristic
            :param req: GetPlan
            :return: Path()
        """
        start = req.start
        goal = req.goal
        self.paint_point(start, goal)


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

    def paint_point(self, start_pose, end_pose):
        """
        Subscriber client to get the published goal point and paint it in rviz
        :param point: goal point
        """
        self.start = start_pose
        start_x = start_pose.pose.point.x
        start_y = start_pose.pose.point.y
        start_quat = start_pose.pose.orientation
        start_euler = euler_from_quaternion(start_quat)
        start_ang = start_euler[2]

        self.goal = end_pose
        end_x = end_pose.pose.point.x
        end_y = end_pose.pose.point.y
        end_quat = end_pose.pose.orientation
        end_euler = euler_from_quaternion(end_quat)
        end_ang = end_euler[2]

        rospy.logdebug("Start x, y, ang: %s %s %s" % (start_x, start_y, start_ang))
        rospy.logdebug("Goal x, y, ang: %s %s %s" % (end_x, end_y, end_ang))

        painted_cell = map_helper.to_grid_cells([(x,y)], self.map)

        self.a_star((start_x, start_y), (end_x, end_y))
        self.point_pub.publish(painted_cell)


    def a_star(self, start, goal):
        """
            A*
            This is where the A* algorithum belongs
            :param start: tuple of start pose
            :param goal: tuple of goal pose
            :return: dict of tuples
        """
        start = map_helper.world_to_index2d(start, self.map)
        goal = map_helper.world_to_index2d(goal, self.map)

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
                    priority = new_cost + self.euclidean_heuristic(goal, next)
                    frontier.put(next, priority)
                    came_from[next] = current

        frontier_list = frontier.get_items()
        frontier_map = []
        frontier_to_display = []
        for point in frontier_list:
            frontier_map.append(map_helper.world_to_index2d(point, self.map))
            frontier_to_display.append(map_helper.index2d_to_world(point, self.map))

        rospy.logdebug("Frontier list: %s " % frontier_list)

        path = [map_helper.index2d_to_world(goal, self.map)]
        last_node = goal
        while came_from[last_node] is not None:
            next_node = came_from[last_node]
            path.insert(0, map_helper.index2d_to_world(next_node, self.map))
            last_node = next_node

        new_path = self.optimize_path(path)
        rospy.logdebug("Path %s " % new_path)

        self.paint_wavefront(frontier_to_display)

        self.paint_obstacles(new_path)
        self.paint_cells(frontier_list, new_path)

    def euclidean_heuristic(self, point1, point2):
        """
            calculate the dist between two points
            :param point1: tuple of location
            :param point2: tuple of location
            :return: dist between two points
        """
        #Pythagorian theorem
        return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)
    pass

    def move_cost(self, current, next):
        """
              calculate the dist between two points
              :param current: tuple of location
              :param next: tuple of location
              :return: dist between two points
        """
        return abs(current[0] - next[0]) + abs(current[1] - next[1])

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
        pathOptimized = []

        for idx in range(len(path)):
            if(idx == 0 or idx == len(path)-1):
                pathOptimized.append(path[idx])
            elif(not self.redundant_point(path[idx - 1], path[idx], path[idx + 1])):
                pathOptimized.append(path[idx])

        return pathOptimized

    def redundant_point(self, last, curr, next):
        if(last[0] == curr[0] == next[0]):
            return True
        if(last[1] == curr[1] == next[1]):
            return True
        return False

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

    def draw_circle(self):
        obstacles = [(math.cos(i/3.0), math.sin(i/3.0)) for i in range(0, 20)]
        rospy.logdebug(obstacles)
        cells = map_helper.to_grid_cells(obstacles, self.map)
        self.obstacles_pub.publish(cells)

    def paint_obstacles(self, obstacles):
        """
        Paints the grids cells in the list to obstacles publisher
        :param obstacles: list of tuples
        :return:
        """
        rospy.logdebug("Painting Path")
        cells = map_helper.to_grid_cells(obstacles, self.map)
        self.obstacles_pub.publish(cells)

    def paint_wavefront(self, wavefront):
        """
        :param wavefront: list of tuples indicating current wavefront
        :return:
        """
        cells = map_helper.to_grid_cells(wavefront, self.map)
        self.wavefront_pub.publish(cells)


if __name__ == '__main__':
    astar = A_Star()
    rospy.loginfo("Initializing A_Star")

    rate = rospy.Rate(1)
    rate.sleep()
    astar.draw_circle()
    while not rospy.is_shutdown():
        # astar.draw_circle()
        # astar.a_star((0, 0), (2, 3))
        rate.sleep()
    rospy.spin()
    pass
