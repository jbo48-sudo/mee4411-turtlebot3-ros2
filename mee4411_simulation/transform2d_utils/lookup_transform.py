from rclpy.duration import Duration
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.time import Time
import tf2_ros

from geometry_msgs.msg import Transform

import numpy as np
from typing import Optional, Tuple, Union

from .transform2d_utils import transform2homogeneous, transform2xyt


def lookup_transform(logger: RcutilsLogger,
                     tf_buffer: tf2_ros.Buffer,
                     base_frame: str,
                     child_frame: str,
                     stamp: Time,
                     time_travel: Optional[Duration] = Duration(seconds=0),
                     format: Optional[str] = 'transform'
                     ) -> Union[None, np.ndarray, Transform, Tuple]:
    """
    Look up transformation on the TF2 buffer from base_frame to child_frame.

    Inputs:
        logger: ROS2 logger object to write output
        tf_buffer: TF2 buffer object
        base_frame: base frame of transformation
        child_frame: child frame of transformation
        stamp: time stamp for TF lookup
        time_travel: amount to move backward in time
        format: format of the returned transformation 'transform', 'homogeneous', or 'xyt'
    Outputs:
        Transformation in the given format
    """
    # Formation validation
    if format not in {'transform', 'homogeneous', 'xyt'}:
        logger.warn(f'Format {format} not valid, must be: transform, homogeneous, or xyt')
        return None

    # Lookup transform
    try:
        trans = tf_buffer.lookup_transform(
            base_frame,
            child_frame,
            stamp - time_travel,
            Duration(seconds=2.0)
        )
    except (tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException) as error:
        logger.warn(f'TF lookup failed: {error}')
        raise

    # Return value
    if format == 'transform':
        return trans
    elif format == 'homogeneous':
        return transform2homogeneous(trans.transform)
    elif format == 'xyt':
        return transform2xyt(trans.transform)
