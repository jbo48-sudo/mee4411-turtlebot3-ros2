from sensor_msgs.msg import JointState

import numpy as np
from typing import List

from tb3_utils import TB3Params


class TB3Kinematics(TB3Params):

    def __init__(self, robot_model) -> None:
        super().__init__(robot_model)

    def calculate_wheel_change(
            self,
            new_joint_states: JointState,
            prev_joint_states: JointState) -> List[float]:
        """
        Calculate the change in wheel angles and time between two joint state messages.

        Inputs:
            new_joint_states    new joint states (sensor_msgs/msg/JointState)
            prev_joint_states   previous joint states (sensor_msgs/msg/JointState)
        Outputs:
            delta_wheel_l       change in left wheel angle [rad]
            delta_wheel_r       change in right wheel angle [rad]
            delta_time          change in time [s]
        """
        ##### YOUR CODE STARTS HERE ##### # noqa: E266
        # TODO Calculate the output values
        # JD:I think list needs index. but need sim to run and get joint names
        # Need to figure out time stuff

        new_lp = new_joint_states.position[0]
        new_rp = new_joint_states.position[1]
        
        new_tsec = new_joint_states.header.stamp.sec
        new_tnano= new_joint_states.header.stamp.nanosec
        new_t = new_tsec + new_tnano * 1e-9

        old_lp = prev_joint_states.position[0]
        old_rp = prev_joint_states.position[1]

        old_tsec = prev_joint_states.header.stamp.sec
        old_tnano = prev_joint_states.header.stamp.nanosec
        old_t = old_tsec + old_tnano * 1e-9

        delta_wheel_l = new_lp - old_lp
        delta_wheel_r = new_rp - old_rp
        delta_time = new_t-old_t
        ##### YOUR CODE ENDS HERE   ##### # noqa: E266

        # Data validation
        if np.isnan(delta_wheel_l):
            delta_wheel_l = 0.0
        if np.isnan(delta_wheel_r):
            delta_wheel_r = 0.0

        return (delta_wheel_l, delta_wheel_r, delta_time)

    def calculate_displacement(
            self,
            delta_wheel_l: float,
            delta_wheel_r: float) -> List[float]:
        """
        Calculate the displacement of the robot based on the change in wheel angles.

        Inputs:
            delta_wheel_l       change in left wheel angle [rad]
            delta_wheel_r       change in right wheel angle [rad]
        Outputs:
            delta_s         linear displacement [m]
            delta_theta     angular displacement [rad]
        """
        ##### YOUR CODE STARTS HERE ##### # noqa: E266
        # TODO Calculate the output values
        # Convert wheel rotation to linear displacement
        d_left = delta_wheel_l * self.wheel_radius
        d_right = delta_wheel_r * self.wheel_radius

        # Linear and angular displacement
        delta_s = (d_right + d_left) / 2.0
        delta_theta = (d_right - d_left) / self.wheel_separation
        ##### YOUR CODE ENDS HERE   ##### # noqa: E266

        return (delta_s, delta_theta)

    def calculate_pose(
            self,
            prev_pose: List[float],
            delta_s: float,
            delta_theta: float) -> List[float]:
        """
        Calculate the new pose of the robot based on the previous pose and the displacement.

        Inputs:
            prev_pose       input pose in format (x, y, theta) [m, m, rad]
            delta_s         linear displacement [m]
            delta_theta     angular displacement [rad]
        Outputs:
            pose            output pose in format (x, y, theta) [m, m, rad]
        """
        ##### YOUR CODE STARTS HERE ##### # noqa: E266
        # TODO Calculate the output values
        x, y, theta = prev_pose
        x_new = x + delta_s * np.cos(theta + 0.5 * delta_theta)
        y_new = y + delta_s * np.sin(theta + 0.5 * delta_theta)
        # TODO fix our new theta
        theta_new =  theta + delta_theta
        
        # old NF attempt. arc motion is correct just more than we need.
        # if abs(delta_theta) < 1e-6:
        #     # Straight motion
        #     x_new = x + delta_s * np.cos(theta)
        #     y_new = y + delta_s * np.sin(theta)
        #     theta_new = theta
        # else:
        #     # Arc motion
        #     # TODO where did this come from what do JD
        #     R = delta_s / delta_theta
        #     x_new = x + R * (np.sin(theta + delta_theta) - np.sin(theta))
        #     y_new = y - R * (np.cos(theta + delta_theta) - np.cos(theta))
        #     theta_new = theta + delta_theta

        pose = x_new , y_new, theta_new  # FILL THIS IN
        ##### YOUR CODE ENDS HERE   ##### # noqa: E266

        return pose
