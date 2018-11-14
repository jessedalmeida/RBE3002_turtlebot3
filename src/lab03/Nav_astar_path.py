#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


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

        self.goal = PoseStamped()

    def odom_callback(self, msg):
        # type: (Odometry) -> None
        """
        update the state of the robot
        :type msg: Odom
        :return:
        """
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        q = [quat.x, quat.y, quat.z, quat.w]
        self.roll, self.pitch, self.yaw = euler_from_quaternion(q)



    def goal_callback(self, msg):
        # type: (PoseStamped) -> None
        """
        Call back function for when a 2D nav goal is published in rviz
        :param goal: PoseStamped
        :return:
        """
        self.goal = msg








if __name__ == '__main__':
    nav_astar_path = Nav_astar_path()

    rospy.loginfo("Initializing Nav_astar_path")

    while not rospy.is_shutdown():
        pass

    rospy.spin()
    pass