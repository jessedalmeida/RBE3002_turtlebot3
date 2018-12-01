"""
    index2d - tuple containing x and y index values for the 2d occupancy grid
    point - tuple containing x and y coordinate values for a 2d real-world map
"""

#!/usr/bin/env python
import sys
import rospy
import math
from nav_msgs.msg import OccupancyGrid, GridCells, Path, MapMetaData
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, PoseStamped, PoseArray, Pose


def get_neighbors(index2d, my_map, occupation=0):
    """
        returns the legal neighbors of index2d
        :param index2d: tuple of index in 2d grid cells
        :param my_map: 1d map array
        :param occupation: only returns if occupation matches the cell contents
        :return: list of tuples
    """

    list_of_neighbors = []

    x_index = index2d[0]
    y_index = index2d[1]

    if is_valid_index2d((x_index, y_index - 1), my_map, occupation):
        neighbor_n = (x_index, y_index - 1)
        list_of_neighbors.append(neighbor_n)

    if is_valid_index2d((x_index + 1, y_index), my_map, occupation):
        neighbor_e = (x_index + 1, y_index)
        list_of_neighbors.append(neighbor_e)

    if is_valid_index2d((x_index, y_index + 1), my_map, occupation):
        neighbor_s = (x_index, y_index + 1)
        list_of_neighbors.append(neighbor_s)

    if is_valid_index2d((x_index - 1, y_index), my_map, occupation):
        neighbor_w = (x_index - 1, y_index)
        list_of_neighbors.append(neighbor_w)

    return list_of_neighbors


def is_valid_index2d(index2d, my_map, occupation=0):
    """
        Gets if a point is a legal location
        :param index2d: tuple of location
        :param my_map: 1d map array
        :param occupation: what value to check for in the cell
        :return: boolean is a legal point
    """
    x_index = index2d[0]
    y_index = index2d[1]

    if x_index < 0 or x_index >= my_map.info.width or y_index < 0 or y_index >= my_map.info.height:
        return False

    cell_val = my_map.data[index2d_to_index1d(index2d, my_map)]
    if cell_val == occupation:
        return True
    else:
        return False


def world_to_index2d(loc, my_map):
    """converts points to the grid"""
    # take in a real world xy location, give back a 2d index
    x_point = loc[0]
    y_point = loc[1]

    x_index_offset = my_map.info.origin.position.x  # Get the x position of the map origin
    y_index_offset = my_map.info.origin.position.y  # Get the y position of the map origin
    x_point -= x_index_offset
    y_point -= y_index_offset

    res = my_map.info.resolution
    x_index = int(x_point / res)
    y_index = int(y_point / res)

    return x_index, y_index


def index2d_to_world(index2d, my_map):
    """convert a 2d index to a point"""
    x_index = index2d[0]
    y_index = index2d[1]

    res = my_map.info.resolution
    x_point = x_index * res
    y_point = y_index * res

    x_index_offset = my_map.info.origin.position.x
    y_index_offset = my_map.info.origin.position.y
    x_point += x_index_offset + res/2
    y_point += y_index_offset + res/2

    return x_point, y_point


def euclidean_distance(point1, point2):
    """
        calculate the dist between two points
        :param point1: tuple of location
        :param point2: tuple of location
        :return: dist between two points
    """
    # Pythagorian theorem
    return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)


def world_to_map(xy, my_map):
    """
        converts a point from the world to the map
        :param xy: tuple of xy float position
        :param my_map: 1d map array
        :return: tuple of converted point
    """
    x_offset = my_map.info.origin.position.x
    y_offset = my_map.info.origin.position.y
    new_x = xy[0] + x_offset
    new_y = xy[1] + y_offset
    return new_x, new_y


def map_to_world(xy, my_map):
    """
        converts a point from the map to the world
        :param xy: touple of xy float position
        :param my_map: 1d map array
        :return: tuple of converted point
    """
    x_offset = my_map.info.origin.position.x
    y_offset = my_map.info.origin.position.y
    new_x = xy[0] + x_offset
    new_y = xy[1] + y_offset
    return new_x, new_y


def to_cells(points, my_map):
    """
        Creates a GridCell() for Rviz display
        :param points: list of tuples
        :param my_map: 1d map array
        :return: GridCell()
    """


def to_grid_cells(cells_to_paint, my_map, to_world=False):
    """
        Creates a GridCell() for Rviz distplay
        :param cells_to_paint: list of tuples
        :param my_map: 1d map array
        :param to_world: whether to transform points to world
        :return: GridCell()
    """
    grid = GridCells()
    grid.header.frame_id = "/map"
    grid.cell_height = my_map.info.resolution
    grid.cell_width = my_map.info.resolution

    grid.cells = []
    # Fill in points
    for index2d in cells_to_paint:
        if to_world:
            index2d = index2d_to_world(index2d, my_map)
        point = Point()
        point.x = index2d[0]
        point.y = index2d[1]
        grid.cells.append(point)

    return grid


def index1d_to_index2d(index1d, my_map):
    width = my_map.info.width

    y = int(index1d / width)
    x = index1d - (y * width)
    return x, y


def index2d_to_index1d(index2d, my_map):
    return index2d[1] * my_map.info.width + index2d[0]


