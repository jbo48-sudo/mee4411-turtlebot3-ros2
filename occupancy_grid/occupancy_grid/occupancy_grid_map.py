from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid

import numpy as np

from .map_conversions import MapConversions
#rev0

class OccupancyGridMap(MapConversions):
    def __init__(self, boundary, resolution, frame_id) -> None:
        super(OccupancyGridMap, self).__init__(boundary, resolution)
        # Set coordinate frame ID
        self.frame_id = frame_id
        # Initialize empty data array (2D array holding values)
        #   In the range [0, 100], representing the probability of occupancy
        #   If a cell is unknown, set to -1
        self.data = np.zeros(self.array_shape)

    @classmethod
    def from_msg(cls, msg: OccupancyGrid):
        """Create an object from an OccupancyGrid msg."""
        ##### YOUR CODE STARTS HERE ##### # noqa: E266
        # TODO Extract boundary, resolution, and frame_id from input message
        boundary = [0., 0., 1., 1.]
        resolution = 1.
        resolution = msg.info.resolution
        xmin = msg.info.origin.position.x
        ymin = msg.info.origin.position.y

        xmax = xmin + msg.info.width * resolution
        ymax = ymin + msg.info.height * resolution
        boundary = [xmin, ymin, xmax, ymax]
        frame_id = msg.header.frame_id
        ##### YOUR CODE ENDS HERE   ##### # noqa: E266

        # Initialize object
        ogm = cls(boundary, resolution, frame_id)

        ##### YOUR CODE STARTS HERE ##### # noqa: E266
        # TODO Update data array in ogm, based on conventions in the __init__ method
        ogm.data = np.array(msg.data, dtype=int).reshape(ogm.array_shape)

        ##### YOUR CODE ENDS HERE   ##### # noqa: E266
        return ogm

    def add_block(self, block: np.array) -> None:
        """
        Add a block to the map stored in self.data.

        Inputs:
            block   np.array in the format (xmin, ymin, xmax, ymax)
        """
        ##### YOUR CODE STARTS HERE ##### # noqa: E266
        # TODO Fill in all the cells that overlap with the block
        # Ensure indices are within the bounds of self.data
        xmin = max(0, xmin)
        ymin = max(0, ymin)
        xmax = min(self.data.shape[1], xmax)
        ymax = min(self.data.shape[0], ymax)

        # Fill the block region in self.data with 1s (or any desired value)
        self.data[ymin:ymax, xmin:xmax] = 1
        ##### YOUR CODE ENDS HERE   ##### # noqa: E266

    def to_msg(self, time: Time) -> OccupancyGrid:
        """
        Convert the OccupancyGridMap object into an OccupancyGrid ROS message.

        Inputs:
            time    current ROS time
        Outputs:
            msg     OccupancyGrid ROS message
        """
        msg = OccupancyGrid()
        ##### YOUR CODE STARTS HERE ##### # noqa: E266
        # TODO Fill in all the fields of the msg using the data from the class
        xmin, ymin, xmax, ymax = block
        # Create a grid of x, y coordinates for all cells overlapping the block
        x_coords = np.arange(xmin, xmax, self.resolution)
        y_coords = np.arange(ymin, ymax, self.resolution)
        # Create a meshgrid to cover the block
        xv, yv = np.meshgrid(x_coords, y_coords)

        # Flatten arrays to pass into xy2sub
        rows, cols = self.xy2sub(xv.flatten(), yv.flatten())

        # Update valid cells in the map to 100 (occupied)
        valid = (rows >= 0) & (cols >= 0)
        self.data[rows[valid], cols[valid]] = 100

        ##### YOUR CODE ENDS HERE   ##### # noqa: E266
        return msg

    def is_occupied(self, x: np.array, y: np.array) -> bool:
        """
        Check whether the given cells are occupied.

        Inputs:
            x           numpy array of x values
            y           numpy array of y values
        Outputs:
            occupied    np.array of bool values
        """
        ##### YOUR CODE STARTS HERE ##### # noqa: E266
        # TODO Check for occupancy in the map based on the input type
        #used ai chat gpt ""
        rows, cols = self.xy2sub(x, y) #Convert coordinates to row/column indices

        occupied = np.zeros_like(x, dtype='bool')
        valid = (rows >= 0) & (cols >= 0) ##check for valid indices
        occupied[valid] = self.data[rows[valid], cols[valid]] > 0 ## Cells occupied if > 0

        ##### YOUR CODE ENDS HERE   ##### # noqa: E266
        return occupied

    def where_occupied(self, format='xy') -> np.array:
        """
        Find the locations of all cells that are occupied.

        Inputs:
            format      requested format of the returned data ('xy', 'rc', 'ind')
        Outputs:
            locations   np.array with the locations of occupied cells in the requested format
        """
        # Check that requested format is valid
        #USED AI Chat GPT ""
        if format not in ('xy', 'rc', 'ind'):
            raise Exception(f'Requested format {format} invalid, must be xy, rc, or ind')
        ##### YOUR CODE STARTS HERE ##### # noqa: E266
        # TODO Check for occupancy in the map based on the input type
        locations = np.zeros(2)
        rows, cols = np.where(self.data > 0)  # row, col indices of occupied cells

        if format == 'rc':
            locations = np.vstack((rows, cols)).T  # stack as Nx2 array of (row, col)
        elif format == 'xy':
            x, y = self.sub2xy(rows, cols)        # convert to (x, y) coordinates
            locations = np.vstack((x, y)).T
        elif format == 'ind':
            locations = self.sub2ind(rows, cols)  # convert to linear indices
        ##### YOUR CODE ENDS HERE   ##### # noqa: E266
        return locations
