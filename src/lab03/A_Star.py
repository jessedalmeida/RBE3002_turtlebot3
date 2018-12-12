#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid, GridCells, Path
from geometry_msgs.msg import PointStamped, Pose, PoseStamped, Point
import map_helper
from PriorityQueue import PriorityQueue
import math
from rbe3002.srv import MakePath
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
        rospy.init_node("a_star", log_level=rospy.INFO)  # start node

        #Setup Map Publishers
        self.waypoints_pub = rospy.Publisher("local_costmap/waypoints", GridCells, queue_size=10)
        self.path_pub = rospy.Publisher("local_costmap/path", Path, queue_size=10)
        self.point_pub = rospy.Publisher("/point_cell", GridCells, queue_size=10)
        self.wavefront_pub = rospy.Publisher("local_costmap/wavefront", GridCells, queue_size=10)

        # Setup service server
        rospy.Service('make_path', MakePath, self.handle_a_star)

        # Set map to none
        self.map = None
       # rospy.logdebug("Initializing A_Star")
        self.unop_path = Path()
        self.goal = PoseStamped()
        self.start = PoseStamped()
        self.pose = Pose()
        self.points = None

        self.rate = rospy.Rate(.5)

        while self.map is None and not rospy.is_shutdown():
            pass

    def handle_a_star(self, req):
        # type: (MakePath) -> (Path, Path)
        """
            service call that uses A* to create a path.
            This can be altered to also grab the orentation for better heuristic
            :param req: GetPlan
            :return: Path()
        """
        rospy.loginfo("Generating Path")

        # Read inputs
        start = req.start
        goal = req.goal
        self.create_map(req.map)
        # rospy.logdebug("Start: %s" % start)
        # rospy.logdebug("Goal: %s" % goal)
        try:
            self.paint_point(start, goal)
            # Path from list of points
            path = self.publish_path(self.points)

            # Safe path
            safe_path = self.safe_path(path)

            # Path until horizon
            horiz_path = self.horizon_path(safe_path)

            return_path = horiz_path
            success = True
            if (len(return_path.poses) == 0):
                success = False

            if (self.pose_distance(return_path.poses[0],return_path.poses[len(return_path.poses)-1]) < 0.1):
                success = False
        except Exception as e:
            rospy.logdebug("Failed to find path")
            rospy.logdebug(e)
            path = Path()
            return_path = Path()
            success = False

        # Return path and horizon path in service call
        return path, return_path, success

    def create_map(self, new_map):

        """
            Get map and set class variables
            :return:
        """
       # rospy.logdebug("Getting map")
        # Update map
        self.map = new_map
        # Show map details for debugging
        #rospy.logdebug("Resolution is: %s" % new_map.info.resolution)
        x_index_offset = self.map.info.origin.position.x
        y_index_offset = self.map.info.origin.position.y
       # rospy.logdebug("Map Origin: x: %s y: %s" % (x_index_offset, y_index_offset))

    def paint_point(self, start_pose, end_pose):
        """
        Takes a start and end pose, calls the A* algorithm and draws grid cells
        :param start_pose: PoseStamped of the starting position
        :param end_pose: PoseStamped of the ending position
        """
        # Extract start point information
        self.start = start_pose
        start_x = start_pose.pose.position.x
        start_y = start_pose.pose.position.y
        sq = start_pose.pose.orientation
        start_quat = [sq.x, sq.y, sq.z, sq.w]
        start_euler = euler_from_quaternion(start_quat)
        start_ang = start_euler[2]
        # Extract end point information
        self.goal = end_pose
        end_x = end_pose.pose.position.x
        end_y = end_pose.pose.position.y
        eq = end_pose.pose.orientation
        end_quat = [eq.x, eq.y, eq.z, eq.w]
        end_euler = euler_from_quaternion(end_quat)
        end_ang = end_euler[2]

        # Print debug information
        # rospy.logdebug("Start x, y, ang: %s %s %s" % (start_x, start_y, start_ang))
        # rospy.logdebug("Goal x, y, ang: %s %s %s" % (end_x, end_y, end_ang))

        # Generate cell for end position
        painted_cell = map_helper.to_grid_cells([(end_x, end_y)], self.map)

        # Call A*
        self.a_star((start_x, start_y), (end_x, end_y))

        # Publish end point
        self.point_pub.publish(painted_cell)

    def a_star(self, start, goal):
        """
            A*
            This is where the A* algorithum belongs
            :param start: tuple of start pose
            :param goal: tuple of goal pose
            :return: dict of tuples
        """
        # Transform start and end
        start = map_helper.world_to_index2d(start, self.map)
        start = map_helper.get_closest_open(start, self.map)
        rospy.logdebug("Starting at: %s, %s" % (start[0], start[1]))
        goal = map_helper.world_to_index2d(goal, self.map)

        # Priority queue for frontier
        frontier = PriorityQueue()
        frontier.put(start, 0)
        # To find path at end
        came_from = {}
        # Track cost to reach each point
        cost_so_far = {}

        # Initially start
        came_from[start] = None
        cost_so_far[start] = 0

        # Loop till finding a path
        while not frontier.empty():
            # Pop best cell from frontier
            current = frontier.get()
            # rospy.logdebug("Current Node %s " % (current, ))

            # Done!
            if current == goal:
                break

            # Add neighbors to frontier
            #rospy.logdebug("Neighbors: %s" % map_helper.get_neighbors_8count(current, self.map))
            for next in map_helper.get_neighbors_8count(current, self.map):
                # rospy.logdebug("Next node %s" % (next,))

                # Find cost
                new_cost = cost_so_far[current] + self.move_cost(current, next)
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self.heuristic(goal, next)
                    frontier.put(next, priority)
                    came_from[next] = current

        # Display frontier cells
        frontier_list = frontier.get_items()
        frontier_map = []
        frontier_to_display = []
        for point in frontier_list:
            frontier_map.append(map_helper.world_to_index2d(point, self.map))
            frontier_to_display.append(map_helper.index2d_to_world(point, self.map))

        # Print debug
        #rospy.logdebug("Frontier list: %s " % frontier_list)

        # Generate path
        self.unop_path = [map_helper.index2d_to_world(goal, self.map)]
        last_node = goal
        while last_node in came_from and came_from[last_node] is not None:
            next_node = came_from[last_node]
            self.unop_path.insert(0, map_helper.index2d_to_world(next_node, self.map))
            last_node = next_node

        # if(len(self.unop_path) < 4):
        #     raise Exception('Path is too short to safely navigate')

        new_path = self.optimize_path(self.unop_path)
        #rospy.logdebug("Path %s " % new_path)

        self.paint_wavefront(frontier_to_display)

        self.paint_obstacles(new_path)
        self.paint_cells(frontier_list, new_path)
        self.points = new_path

    def heuristic(self, point1, point2):
        """
            calculate the dist between two points
            :param point1: tuple of goal location
            :param point2: tuple of next location
            :return: dist between two points
        """
        #Pythagorian theorem
        euclidean = map_helper.euclidean_distance(point1, point2)

        index1d = map_helper.index2d_to_index1d(point2, self.map)

        occupied_val = float(self.map.data[index1d]/10.0)

        return euclidean + occupied_val

    def move_cost(self, current, next):
        """
              calculate the dist between two points
              :param current: tuple of location
              :param next: tuple of location
              :return: dist between two points
        """
        index1d = map_helper.index2d_to_index1d(next, self.map)
        occupied_val = float(self.map.data[index1d]/10.0)
        return self.straight_line_dist(current, next) + occupied_val

    def straight_line_dist(self, current, next):
        """
              calculate the dist between two points
              :param current: tuple of location
              :param next: tuple of location
              :return: dist between two points
        """
        return abs(current[0] - next[0]) + abs(current[1] - next[1])

    def pose_distance(self, current, next):
        """
        Calculate distance between two poses
        :param current: PoseStamped of first pose
        :param next: PoseStamped of second pose
        :return: straight line distance
        """
        point1 = (current.pose.position.x, current.pose.position.y)
        point2 = (next.pose.position.x, next.pose.position.y)
        return map_helper.euclidean_distance(point1, point2)

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
        return path
        pathOptimized = []

        for idx in range(len(path)):
            if idx == 0 or idx == len(path) - 1:
                pathOptimized.append(path[idx])
            elif not self.redundant_point(path[idx - 1], path[idx], path[idx + 1]):
                pathOptimized.append(path[idx])

        return pathOptimized

    def redundant_point(self, last, curr, next):
        """
        Determines whether a point is redundant
        :param last: preceding point
        :param curr: point to check for redundancy
        :param next: following point
        :return: return whether the point is directly between adjacent points
        """
        # cleaning straight lines
        if last[0] == curr[0] == next[0]:
            return True
        if last[1] == curr[1] == next[1]:
            return True

        # cleaning diagonal lines
        res = self.map.info.resolution
        # if diag in first quadrant
        if (last[0] + res == curr[0] == next[0] - res) and (last[1] + res == curr[1] == next[1] - res):
            return True
        # if diag in second quadrant
        if (last[0] - res == curr[0] == next[0] + res) and (last[1] + res == curr[1] == next[1] - res):
            return True
        # if diag in third quadrant
        if (last[0] - res == curr[0] == next[0] + res) and (last[1] - res == curr[1] == next[1] + res):
            return True
        # if diag in fourth quadrant
        if (last[0] + res == curr[0] == next[0] - res) and (last[1] - res == curr[1] == next[1] + res):
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
        path_poses = []
        for point in points:
            # Generate pose
            pose = PoseStamped()
            # Mark frame
            pose.header.frame_id = "/odom"
            # Populate pose
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            path_poses += [pose]
        # Make path message
        path = Path()
        path.header.frame_id = "/odom"
        path.poses = path_poses
        # Publish to rviz
        self.path_pub.publish(path)
        # Return to send back in service reply
        return path

    def horizon_path(self, path):
        """
        Takes in a path from a start point to an end point. Returns the waypoints along the first horizon distance of the path
        :param path: Path()
        :return: Path()
        """
        horizon_dist = 0.3
        traveled_dist = 0
        horizon_path = Path()
        horizon_path.header.frame_id = "/odom"
        horizon_path.poses.append(path.poses[0])

        for idx in range(len(path.poses) - 1):
            dist = self.pose_distance(path.poses[idx], path.poses[idx + 1])
            traveled_dist += dist
            if traveled_dist < horizon_dist:
                horizon_path.poses.append(path.poses[idx + 1])
            else:
                remaining_dist = horizon_dist - traveled_dist
                horizon_pose = self.pose_btw_poses(path.poses[idx], path.poses[idx + 1], remaining_dist)
                horizon_path.poses.append(horizon_pose)
                break

        return horizon_path

    def safe_path(self, path):
        safe_dist = 0.2
        safe_path = Path()
        safe_path.header.frame_id = "/odom"
        safe_path.poses.append(path.poses[0])

        if self.pose_distance(path.poses[0], path.poses[len(path.poses)-1]) < safe_dist:
            # raise Exception('Starting too close to end to safely navigate')
            return Path()

        for idx in range(len(path.poses)-1):
            dist_next_to_end = self.pose_distance(path.poses[idx+1], path.poses[len(path.poses)-1])
            if dist_next_to_end >= safe_dist:
                safe_path.poses.append(path.poses[idx+1])
            else:
                dist_curr_to_end = self.pose_distance(path.poses[idx], path.poses[len(path.poses)-1])
                remaining_dist = dist_curr_to_end - safe_dist
                if remaining_dist < 0:
                    break
                safe_pose = self.pose_btw_poses(path.poses[idx], path.poses[idx+1], remaining_dist)
                safe_path.poses.append(safe_pose)
                break

        return safe_path



    def pose_btw_poses(self, start_pose, end_pose, dist):
        """
        takes in two poses and a distance, and returns a pose that is between the two poses, the desired distance away from the starting pose
        :param start_pose: PoseStamped()
        :param end_pose: PoseStamped()
        :param dist: double
        :return: PoseStamped()
        """
        intermediate_pose = PoseStamped()
        #intermediate_pose.header.frame_id = start_pose.header.frame_id

        theta = math.atan2(end_pose.pose.position.y - start_pose.pose.position.y, end_pose.pose.position.x - start_pose.pose.position.x)

        intermediate_pose.pose.position.x = start_pose.pose.position.x + dist * math.cos(theta)
        intermediate_pose.pose.position.y = start_pose.pose.position.y + dist * math.sin(theta)

        return intermediate_pose

    def draw_circle(self):
        obstacles = [(math.cos(i/3.0), math.sin(i/3.0)) for i in range(0, 20)]
        # rospy.logdebug(obstacles)
        cells = map_helper.to_grid_cells(obstacles, self.map)
        self.waypoints_pub.publish(cells)

    def paint_obstacles(self, obstacles):
        """
        Paints the grids cells in the list to obstacles publisher
        :param obstacles: list of tuples
        :return:
        """
      #  rospy.logdebug("Painting Path")
        cells = map_helper.to_grid_cells(obstacles, self.map)
        self.waypoints_pub.publish(cells)

    def paint_wavefront(self, wavefront):
        """
        :param wavefront: list of tuples indicating current wavefront
        :return:
        """
        cells = map_helper.to_grid_cells(wavefront, self.map)
        self.wavefront_pub.publish(cells)


if __name__ == '__main__':
    astar = A_Star()
    #rospy.loginfo("Initializing A_Star")

    rate = rospy.Rate(1)
    rate.sleep()
    # Draw the circle
    astar.draw_circle()
    while not rospy.is_shutdown():
        # astar.draw_circle()
        # astar.a_star((0, 0), (2, 3))
        rate.sleep()
    rospy.spin()
    pass