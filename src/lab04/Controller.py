#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion
from rbe3002.srv import MakePath, RobotNav, FrontierRequest
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'lab03'))
import map_helper

class Controller:

    def __init__(self):
        """
            This node explores frontiers until no frontiers remain
        """
        rospy.loginfo("Initializing Controller")

        # Initialize node
        rospy.init_node('controller', log_level=rospy.DEBUG)

        # Subscribers
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        # Setup service proxys
        self.make_path = rospy.ServiceProxy('make_path', MakePath)
        self.robot_nav = rospy.ServiceProxy('robot_nav', RobotNav)
        self.frontier_request = rospy.ServiceProxy('get_frontiers', FrontierRequest)

        self.sub_goal = rospy.Subscriber("move_base_simple/goal", PoseStamped, self.nav_to_point)

        self.goal = PoseStamped()

        self.pose = None

        while(self.pose is None):
            rospy.sleep(20)
        self.start_pose = self.pose

        self.state_machine("Explore")

    def odom_callback(self, msg):
        # type: (Odometry) -> None
        """
        update the state of the robot
        :type msg: Odom
        :return:
        """
        position = msg.pose.pose.position
        new_pose = PoseStamped()
        new_pose.pose.position = position
        self.pose = new_pose

    def nav_to_point(self, goal):
        frontier_request_response = self.frontier_request()
        map = frontier_request_response.map
        path_response = self.make_path(self.pose, goal, map)
        if path_response.success:
            self.robot_nav(path_response.path)

    def explore(self):

        path_found = False
        done_exploring = False
        path_poses = Path().poses
        
        if self.pose is None:
            rospy.loginfo("No known pose!")
            return

        rospy.loginfo("Expanding Map and Updating Frontiers")
        frontier_request_response = self.frontier_request()
        rospy.loginfo("Found Frontiers")
        map = frontier_request_response.map
        frontiers = frontier_request_response.frontiers

        for frontier in frontiers:
            path_response = self.make_path(self.pose, frontier, map)
            # rospy.logdebug("Response: %s" % path_response)
            # rospy.logdebug("Successful path: %s" % path_response.success)
            rospy.loginfo("Potential Path points: ")
            for pose in path_response.path.poses:
                rospy.loginfo(pose.pose.position)
            rospy.loginfo("Potential Horizon Path points: ")
            for pose in path_response.horiz_path.poses:
                rospy.loginfo(pose.pose.position)
            if path_response.success:
                path_found = True
                path_poses = path_response.horiz_path.poses
                #rospy.logdebug("Successful, going to path %s" % path_response.horiz_path.poses)
                break

        if not path_found:
            done_exploring = True
        else:

            rospy.logdebug("Trying to go to a frontier")
            if not self.robot_nav(path_response.horiz_path):
                rospy.logwarn("Robot navigation failed")
                return
            rospy.logdebug("At point %s" % self.pose)

        if done_exploring:
            rospy.loginfo("Done Exploring")
        return done_exploring

    def state_machine(self, state):
        if(state == "Explore"):
            rospy.logdebug("In Explore State")
            while not rospy.is_shutdown() and not self.explore():
                pass
            self.save_map()
            state = "Home"
        if(state == "Home"):
            rospy.logdebug("In Home State")
            self.nav_to_point(self.start_pose)
            state = "Travel"
        if(state == "Travel"):
            rospy.logdebug("In Travel State")
            while not rospy.is_shutdown():
                pass

    def save_map(self):
        os.system("rosrun map_server map_saver -f maps/made_map")

if __name__ == '__main__':
    controller = Controller()

    # rate = rospy.Rate(1000)
    # rate.sleep()
    # while not rospy.is_shutdown() and not controller.explore():
    #     pass

    rospy.spin()