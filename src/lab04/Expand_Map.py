#!/usr/bin/env python
import sys
import rospy
from nav_msgs.msg import OccupancyGrid, GridCells, Path, MapMetaData
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, PoseStamped, PoseArray, Pose



class Expand_Map:

    def __init__(self):
        """
        Use this node to expand the map to ensure that the turtlebot will not enter 
        a space too small for it to enter.
        """

        # Initialize node
        rospy.init_node("expand_map", log_level=rospy.DEBUG)

        # Setup Map Subscriber
        rospy.Subscriber("map", OccupancyGrid, self.map_callback)

        self.map = None

        self.rate = rospy.Rate(.5)

        while self.map is None and not rospy.is_shutdown():
            pass

    def map_callback(self, msg):
        """
            This is a callback for the /map topic
            :param msg: map
            :return: None
        """
        expanded_map = self.expand(msg)
        occo_map = OccupancyGrid()
        occo_map.header = msg.header
        occo_map.data = expanded_map
        occo_map.info = msg.info
        self.expanded_map = occo_map
        self.map_pub.publish(occo_map)


    def handle_map(self, req):

        """
            Service call to get map and expand it
            :return:
        """



    def expand(self,my_map):
        """
            Expand the map and return it
            :param my_map: map
            :return: map
        """
        pass

      

if __name__ == '__main__':

    Expand_Map()
