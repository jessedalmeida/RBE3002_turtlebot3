#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from rbe3002.srv import MakePath, RobotNav


class Nav_astar_path:

    def __init__(self):
        """
            This node creates an omptimum path to travel to and then navigates that path

        """

        # Initialize node
        rospy.init_node('nav_astar_path', log_level=rospy.DEBUG)

        # Subscribers
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback)

        # Setup service proxys
        self.make_path = rospy.ServiceProxy('make_path', MakePath)
        self.robot_nav = rospy.ServiceProxy('robot_nav', RobotNav)

        self.goal = PoseStamped()

        self.pose = None

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

    def goal_callback(self, goal):
        # type: (PoseStamped) -> None
        """
        Call back function for when a 2D nav goal is published in rviz
        :param goal: PoseStamped
        :return:
        """
        if self.pose is None:
            rospy.logwarn("No known pose!")
            return

        self.goal = goal

        # path_response = self.make_path(self.pose, self.goal)
        path_response = self.make_path(self.pose, goal)
        poses = path_response.path
        rospy.logdebug("Response was %s" % path_response)
        rospy.logdebug("Response path was %s" % path_response.path)

        for pose in poses[1:-1]:
            if not self.robot_nav(pose, True):
                rospy.logwarn("Robot navigation failed")
                return
        self.robot_nav(goal, False)


if __name__ == '__main__':
    nav_astar_path = Nav_astar_path()

    rospy.loginfo("Initializing Nav_astar_path")

    while not rospy.is_shutdown():
        pass

    rospy.spin()
    pass