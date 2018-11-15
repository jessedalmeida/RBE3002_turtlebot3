#!/usr/bin/env python
import rospy
import tf
import sys, select, termios, tty
import math
import time
import numpy
from geometry_msgs.msg import PoseStamped, PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from rbe3002.srv import RobotNav

class Robot:
    MAX_ANG_VEL = 1.0
    MAX_LIN_VEL = 0.2

    def __init__(self):
        """"
        Set up the node here
        """
        # Init node
        rospy.init_node('robot_drive_controller')
        # Setup ros publishers
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        # Setup subscribers
        self.sub_odom = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.sub_goal = rospy.Subscriber("/goal", PoseStamped, self.nav_to_pose)
        # Setup service client
        self.srv_nav = rospy.Service("robot_nav", RobotNav, self.handle_robot_nav)

    def handle_robot_nav(self, req):
        """
        Handler for robot_nav service
        :param req: RobotNav request
        :return:
        """
        # type: (RobotNav) -> None
        goal = req.goal
        ignore_orientation = req.ignoreOrientation
        # Determine if the orientation needs to be ignored
        if(ignore_orientation):
            self.nav_to_point(goal)
            return True
        else:
            self.nav_to_pose(goal)
            return True

    def nav_to_pose(self, goal):
        # type: (PoseStamped) -> None
        """
        This is a callback function. It should extract data from goal, drive in a striaght line to reach the goal and
        then spin to match the goal orientation.
        :param goal: PoseStamped
        :return:
        """

        goalX = goal.pose.position.x
        goalY = goal.pose.position.y
        goalQuat = (goal.pose.orientation.x,
                goal.pose.orientation.y,
                goal.pose.orientation.z,
                goal.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(goalQuat)
        goalAng = euler[2]

        distToGoal = math.sqrt(math.pow(goalX - self.px, 2) + math.pow(goalY - self.py, 2))

        angToGoal = math.atan2(goalY - self.py, goalX - self.px) - self.yaw

        self.rotate(angToGoal)
        self.drive_straight(Robot.MAX_LIN_VEL, distToGoal)
        self.rotate(-self.yaw + goalAng)

    def nav_to_point(self, goal):
        # type: (PoseStamped) -> None
        """
        :param goal: PoseStamped
        :return:
        """
        goalX = goal.pose.position.x
        goalY = goal.pose.position.y

        distToGoal = math.sqrt(math.pow(goalX - self.px, 2) + math.pow(goalY - self.py, 2))

        angToGoal = math.atan2(goalY - self.py, goalX - self.px) - self.yaw

        self.rotate(angToGoal)
        self.drive_straight(Robot.MAX_LIN_VEL, distToGoal)

    def drive_straight(self, speed, distance):
        """
        Make the turtlebot drive straight
        :type speed: float
        :type distance: float
        :param speed: speed to drive
        :param distance: distance to drive
        :return:
        """

        twist = Twist()
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        twist.linear.z = 0.0
        twist.linear.y = 0.0

        if(speed > Robot.MAX_LIN_VEL):
            speed = Robot.MAX_LIN_VEL

        avgSpeed = speed/2.0
        t0 = rospy.Time.now().secs
        tf = t0 + (distance/avgSpeed)
        dt = tf-t0
        p0 = 0
        pf = distance
        v0 = 0
        vf = 0
        a0 = 0
        af = 0

        aVals = self.aVals(0, dt, p0, pf, v0, vf, a0, af)
        a1 = aVals[1][0]
        a2 = aVals[2][0]
        a3 = aVals[3][0]
        a4 = aVals[4][0]
        a5 = aVals[5][0]

        # 5ms loop rate
        rate = rospy.Rate(5)
        while(rospy.Time.now().secs < tf and not rospy.is_shutdown()):
            t = rospy.Time.now().secs - t0
            twist.linear.x = a1 + 2*a2*t + 3*a3*pow(t, 2) + 4*a4*pow(t, 3) + 5*a5*pow(t, 4)
            self.pub.publish(twist)
            rate.sleep()

        twist.linear.x = 0.0
        self.pub.publish(twist)

    def rotate(self, angle):
        """
        Rotate in place
        :param angle: angle to rotate
        :return:
        """
        # Set update rate
        rate = rospy.Rate(10)

        # Initial angle
        yaw = self.yaw
        start = yaw

        dest_ang = self.bounded_angle(start + angle)
        rospy.loginfo("Turning angle: %f" % (angle))
        error = self.bounded_angle(dest_ang - yaw)

        # Loop while not there yet
        while not rospy.is_shutdown() and \
                abs(error) > .03:
            cmd = Twist()
            cmd.linear.x = 0
            error = self.bounded_angle(dest_ang - yaw)
            # Proportional + feed forward control
            cmd.angular.z = error * (.3 + .1/abs(error))
            self.pub.publish(cmd)
            # Update position
            yaw = self.yaw
            rate.sleep()
        # Stop robot
        cmd = Twist()
        self.pub.publish(cmd)
        rospy.loginfo("Done turning")

    def oldrotate(self, angle):
        """
        Rotate in place
        :param angle: angle to rotate
        :return:
        """

        twist = Twist()
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        startAng = self.yaw
        endAng = startAng + angle
        endAng = endAng - 2 * math.pi if endAng > math.pi else endAng
        endAng = endAng + 2 * math.pi if endAng < -math.pi else endAng

        # 5ms loop rate
        rate = rospy.Rate(5)
        if (startAng < endAng):
            while self.yaw < endAng and not rospy.is_shutdown():
                twist.angular.z = Robot.MAX_ANG_VEL
                self.pub.publish(twist)
                rate.sleep()
        else:
            while self.yaw > endAng and not rospy.is_shutdown():
                twist.angular.z = -Robot.MAX_ANG_VEL
                self.pub.publish(twist)
                rate.sleep()

        twist.angular.z = 0.0
        self.pub.publish(twist)

    def bounded_angle(self, angle):
        """
        Corrects angle wrap, by moving it between -pi and pi
        :param angle: Input angle in radians
        :return: Angle between pi and -pi radians
        """
        return ((angle + math.pi) % (2 * math.pi)) - math.pi

    def aVals(self, t0, tf, p0, pf, v0, vf, a0, af):
        """
        Helper for trajectory planning
        :param t0: Initial time
        :param tf: End time
        :param p0: Initial position
        :param pf: Final position
        :param v0: Initial velocity
        :param vf: Final velocity
        :param a0: Initial acceleration
        :param af: Final acceleration
        :return: parameters for trajectory
        """
        tMat = numpy.array([[1, t0, pow(t0, 2), pow(t0, 3), pow(t0, 4), pow(t0, 5)],
                            [0, 1, 2 * t0, 3 * pow(t0, 2), 4 * pow(t0, 3), 5 * pow(t0, 4)],
                            [0, 0, 2, 6 * t0, 12 * pow(t0, 2), 20 * pow(t0, 3)],
                            [1, tf, pow(tf, 2), pow(tf, 3), pow(tf, 4), pow(tf, 5)],
                            [0, 1, 2 * tf, 3 * pow(tf, 2), 4 * pow(tf, 3), 5 * pow(tf, 4)],
                            [0, 0, 2, 6 * tf, 12 * pow(tf, 2), 20 * pow(tf, 3)]],
                           dtype='float')

        pvaMat = numpy.array([[p0], [v0], [a0], [pf], [vf], [af]])

        tMatInv = numpy.linalg.inv(tMat)

        aMat = numpy.matmul(tMatInv,pvaMat)

        return aMat

    def odom_callback(self, msg):

        """
        update the state of the robot
        :type msg: Odom
        :return:
        """
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        self.quat = [0] * 4
        self.quat[0] = msg.pose.pose.orientation.x
        self.quat[1] = msg.pose.pose.orientation.y
        self.quat[2] = msg.pose.pose.orientation.z
        self.quat[3] = msg.pose.pose.orientation.w
        euler = tf.transformations.euler_from_quaternion(self.quat)
        self.yaw = euler[2]


if __name__ == '__main__':

    bot = Robot()
    time.sleep(1)

    #bot.rotate(math.pi)
    #bot.drive_straight(1,.5)

    rospy.spin()