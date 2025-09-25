#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid  # noqa: F401
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

from .map_conversions import MapConversions  # noqa: F401
from .occupancy_grid_map import OccupancyGridMap  # noqa: F401
#rev0

class OccupancyGridNode(Node):

    def __init__(self):
        super().__init__('occupancy_grid')
        ##### YOUR CODE STARTS HERE ##### # noqa: E266
        # TODO Set up the ROS node
        qos_profile = QoSProfile(depth=1)
        qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher_ = self.create_publisher(OccupancyGrid, 'map', qos_profile)

        # TODO Set up the ROS publisher for the occupancy grid map
        self.publisher_ = self.create_publisher(OccupancyGrid, 'map', qos_profile)

        # TODO Read in the map information from the ROS parameter server
        self.declare_parameter('boundary', [0.0, 0.0, 5.0, 5.0])
        self.declare_parameter('map_resolution', 1.0)
        self.declare_parameter('frame_id', 'map')

        boundary = self.get_parameter('boundary').get_parameter_value().double_array_value
        resolution = self.get_parameter('map_resolution').get_parameter_value().double_value
        frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        # TODO Create an OccupancyGridMap based on the provided data using occupancy_grid_utils
        self.map_ = OccupancyGridMap(boundary, resolution, frame_id)

        '''
        prof added below code as a possible way to simplify our message output class has built in methods
        '''
        ## map_msg = self.map_.to_msg([TIME STAMP])

        # TODO Create and publish a nav_msgs/OccupancyGrid msg
        msg = OccupancyGrid()
        msg.header.frame_id = frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = resolution
        msg.info.width = self.map_.array_shape[1]  # number of columns
        msg.info.height = self.map_.array_shape[0]  # number of rows
        msg.info.origin.position.x = boundary[0]
        msg.info.origin.position.y = boundary[1]
        msg.info.origin.position.z = 0.0
        # Orientation (no rotation)
        msg.info.origin.orientation.w = 1.0

        # Flatten the 2D numpy map to row-major order
        msg.data = self.map_.data.flatten().astype(int).tolist()

        self.publisher_.publish(msg)
        self.get_logger().info('Published OccupancyGrid message')

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
