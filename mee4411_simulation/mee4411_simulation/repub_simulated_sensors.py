#!/usr/bin/env python3
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from sensor_msgs.msg import LaserScan, JointState

from copy import deepcopy
import numpy as np


class SimRepub(Node):

    def __init__(self) -> None:
        super().__init__('repub_simulated_sensors')
        # Declare paramters
        self.declare_parameter('namespace_remove',
                               value='',
                               descriptor=ParameterDescriptor(
                                   type=ParameterType.PARAMETER_STRING))
        self.declare_parameter('noise_std',
                               value=0.0,
                               descriptor=ParameterDescriptor(
                                   type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter('alpha',
                               value=0.9,
                               descriptor=ParameterDescriptor(
                                   type=ParameterType.PARAMETER_DOUBLE))

        # Namespace to remove from frames
        self.namespace_remove_ = \
            self.get_parameter('namespace_remove').get_parameter_value().string_value
        self.noise_std_ = self.get_parameter('noise_std').get_parameter_value().double_value
        self.alpha_ = \
            self.get_parameter('alpha').get_parameter_value().double_value  # running average param

        # ROS publishers and subscribers
        self.scan_pub_ = self.create_publisher(LaserScan, '~/scan_out', 10)
        self.joint_states_pub_ = self.create_publisher(JointState, '~/joint_states_out', 10)
        self.prev_joint_state_ = None  # previous joint states
        self.joint_state_rate_ = None  # rate at which messages are being published
        self.position_ = [0.0, 0.0]
        self.velocity_ = [0.0, 0.0]
        self.get_clock().sleep_for(Duration(seconds=0.01))
        self.scan_sub_ = self.create_subscription(LaserScan, 'scan', self.scanCallback, 1)
        self.joint_states_sub_ = self.create_subscription(
            JointState,
            'joint_states',
            self.jointStateCallback,
            1)

    def stripName(self, name: str) -> str:
        # Strip "self.namespace_remove_" from "name"
        parts = name.split('/')
        parts = [p for p in parts if p not in ['', self.namespace_remove_]]
        return "/".join(parts)

    def scanCallback(self, msg: LaserScan) -> None:
        msg.header.frame_id = self.stripName(msg.header.frame_id)
        self.scan_pub_.publish(msg)

    def jointStateCallback(self, msg: JointState) -> None:
        msg_out = deepcopy(msg)

        # Change names
        msg_out.header.frame_id = self.stripName(msg.header.frame_id)
        msg_out.name = [self.stripName(n) for n in msg.name]

        # Add noise (only if moving)
        if self.noise_std_ > 0.0 and any([abs(v) > 0 for v in msg.velocity]):
            if self.prev_joint_state_ is not None:
                # Get timing information
                dt = (msg.header.stamp - self.prev_joint_state_.header.stamp).to_sec()
                if self.joint_state_rate_ is None:
                    self.joint_state_rate_ = dt
                else:
                    self.joint_state_rate_ = self.alpha_ * self.joint_state_rate_ \
                        + (1-self.alpha_) * dt
                # Get change in position
                delta = [p - q + np.random.randn() * self.noise_std_ * self.joint_state_rate_
                         for p, q in zip(msg.position, self.prev_joint_state_.position)]
                self.position_ = [p + d for p, d in zip(self.position_, delta)]
                msg_out.position = self.position_
                msg_out.velocity = [v + np.random.randn()*self.noise_std_*self.joint_state_rate_
                                    for v in msg.velocity]
            self.prev_joint_state_ = msg

        # Publish message
        self.joint_states_pub_.publish(msg_out)


def main(args=None):
    # Start up ROS2
    rclpy.init(args=args)

    # Create the node
    srnode = SimRepub()

    # Let the node run until it is killed
    rclpy.spin(srnode)

    # Clean up the node and stop ROS2
    srnode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
