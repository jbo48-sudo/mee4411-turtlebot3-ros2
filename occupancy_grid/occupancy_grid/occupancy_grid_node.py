#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid  # noqa: F401

from .map_conversions import MapConversions  # noqa: F401
from .occupancy_grid_map import OccupancyGridMap  # noqa: F401


class OccupancyGridNode(Node):

    def __init__(self):
        super().__init__('occupancy_grid')
        ##### YOUR CODE STARTS HERE ##### # noqa: E266
        # TODO Set up the ROS node
        pass

        # TODO Set up the ROS publisher for the occupancy grid map
        pass

        # TODO Read in the map information from the ROS parameter server
        pass

        # TODO Create an OccupancyGridMap based on the provided data using occupancy_grid_utils
        pass

        # TODO Create and publish a nav_msgs/OccupancyGrid msg
        pass
        ##### YOUR CODE ENDS HERE   ##### # noqa: E266


def main(args=None):
    # Start up ROS2
    rclpy.init(args=args)

    # Create the node
    og_node = OccupancyGridNode()

    # Let the node run until it is killed
    rclpy.spin(og_node)

    # Clean up the node and stop ROS2
    og_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
