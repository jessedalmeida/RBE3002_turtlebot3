#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from rbe3002.srv import MakePath, RobotNav, FrontierRequest


class Controller:

    def __init__(self):
        """
            This node explores frontiers until no frontiers remain
        """

        # Initialize node
        rospy.init_node('controller', log_level=rospy.DEBUG)

        # Subscribers
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        # Setup service proxys
        self.make_path = rospy.ServiceProxy('make_path', MakePath)
        self.robot_nav = rospy.ServiceProxy('robot_nav', RobotNav)
        self.frontier_request = rospy.ServiceProxy('frontier_request', FrontierRequest)

        self.goal = PoseStamped()

        self.pose = None

        self.explore()

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

    def explore(self):

        done_exploring = False
        
        if self.pose is None:
            rospy.logwarn("No known pose!")
            return

        frontier_request_response= self.frontier_request()
        map = frontier_request_response.map
        frontiers = frontier_request_response.frontiers

        for frontier in frontiers:
            path_response = self.make_path(self.pose, frontier, map)
            if path_response.success:
                break
        if not path_response.success:
            done_exploring = True
        else:
            poses = path_response.horiz_path.poses
            for pose in poses[1:-1]:
                if not self.robot_nav(pose, True):
                    rospy.logwarn("Robot navigation failed")
                    return
                rospy.logdebug("At point %s" % self.pose)
            self.robot_nav(poses[-1], False)

        return done_exploring

if __name__ == '__main__':
    controller = Controller()

    rospy.loginfo("Initializing Controller")

    while not rospy.is_shutdown():
        while(not controller.explore()):
            pass

    rospy.spin()
    pass