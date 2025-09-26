#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

from numpy import arange, ceil


class BlockVisualizer(Node):

    def __init__(self):
        super().__init__('block_visualizer')

        # initialize publisher
        latching_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        blocks_pub = self.create_publisher(MarkerArray, 'blocks', qos_profile=latching_qos)
        grid_pub = self.create_publisher(Marker, 'grid_lines', qos_profile=latching_qos)
        # https://docs.ros2.org/foxy/api/nav_msgs/msg/GridCells.html

        # initialize parameters
        self.declare_parameter('boundary',
                               descriptor=ParameterDescriptor(
                                   type=ParameterType.PARAMETER_DOUBLE_ARRAY))
        self.declare_parameter('blocks',
                               descriptor=ParameterDescriptor(
                                   type=ParameterType.PARAMETER_DOUBLE_ARRAY))
        self.declare_parameter('frame_id',
                               descriptor=ParameterDescriptor(
                                   type=ParameterType.PARAMETER_STRING))
        self.declare_parameter('alpha', 0.5,
                               descriptor=ParameterDescriptor(
                                   type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('resolution',
                               descriptor=ParameterDescriptor(
                                   type=ParameterType.PARAMETER_DOUBLE))

        # get parameter values
        boundary = self.get_parameter('boundary').get_parameter_value().double_array_value
        blocks = self.get_parameter('blocks').get_parameter_value().double_array_value
        blocks = [blocks[i:i+4] for i in range(0, len(blocks), 4)]  # reshape the block array
        alpha = self.get_parameter('alpha').get_parameter_value().double_value
        resolution = self.get_parameter('resolution').get_parameter_value().double_value

        # print map information to screen
        self.get_logger().info("boundary: {:.2f}, {:.2f}, {:.2f}, {:.2f}".format(*boundary))
        for block in blocks:
            self.get_logger().info("block: {:.2f}, {:.2f}, {:.2f}, {:.2f}".format(*block))

        # initialize header
        h = Header()
        h.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        h.stamp = self.get_clock().now().to_msg()

        # initialize block marker message
        ma = []  # marker array

        # initialize boundary wall blocks
        thickness = 0.05  # thickness of walls
        height = 0.5  # height of markers
        for i in range(0, 4):
            m = Marker()
            m.header = h
            m.ns = "boundary"
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            # ensure all area inside of the boundary is in the map
            if i % 2 == 0:
                m.pose.position.x = (boundary[0]+boundary[2])/2.
                m.pose.position.y = boundary[i+1] - (-1)**(i/2.) * thickness/2.
                m.scale.x = boundary[2] - boundary[0] + 2. * thickness
                m.scale.y = thickness
            else:
                m.pose.position.x = boundary[i-1] - (-1)**((i-1.)/2.) * thickness/2.
                m.pose.position.y = (boundary[1]+boundary[3])/2.
                m.scale.x = thickness
                m.scale.y = boundary[3] - boundary[1] + 2. * thickness
            m.pose.position.z = height/2.
            m.pose.orientation.w = 1.0
            m.scale.z = height
            m.color.r = 157./255.  # Temple cherry color
            m.color.g = 34./255.
            m.color.b = 53./255.
            m.color.a = 1.0

            ma.append(m)

        # initialize block markers
        for i, b in enumerate(blocks):
            m = Marker()
            m.header = h
            m.ns = "blocks"
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = (b[0]+b[2]) / 2.
            m.pose.position.y = (b[1]+b[3]) / 2.
            m.pose.position.z = height/2.
            m.pose.orientation.w = 1.
            m.scale.x = b[2] - b[0]
            m.scale.y = b[3] - b[1]
            m.scale.z = height
            m.color.r = 157./255.  # Temple cherry color
            m.color.g = 34./255.
            m.color.b = 53./255.
            m.color.a = alpha  # make slightly transparent

            ma.append(m)

        blocks_pub.publish(MarkerArray(markers=ma))

        # Create GridCells message
        m = Marker()
        m.header = h
        m.ns = "grid_lines"
        m.id = i
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD
        m.pose.orientation.w = 1.
        m.scale.x = 0.01  # line width
        m.color.r = 128./255.
        m.color.g = 128./255.
        m.color.b = 128./255.
        m.color.a = 1.0

        # Populate the grid cells
        xmin = boundary[0]
        xmax = boundary[0]+ceil((boundary[2]-boundary[0])/resolution)*resolution
        ymin = boundary[1]
        ymax = boundary[1]+ceil((boundary[3]-boundary[1])/resolution)*resolution
        for x in arange(xmin, xmax+1.e-6, resolution):
            m.points.append(Point(x=x, y=ymin, z=0.0))
            m.points.append(Point(x=x, y=ymax, z=0.0))
        for y in arange(ymin, ymax+1.e-6, resolution):
            m.points.append(Point(x=xmin, y=y, z=0.0))
            m.points.append(Point(x=xmax, y=y, z=0.0))

        # Publish the GridCells message
        grid_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)

    block_viz = BlockVisualizer()

    rclpy.spin(block_viz)

    block_viz.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
